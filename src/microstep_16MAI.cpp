/*lexique:  BP= bouton pousoir
            M = moteur
            V = vérin
            E = état
            C = capteur
            FC = fin de course
            pos = position
            HV = high value: c est pour les compteur de pas à pas
            LV = low value
            D = direction pour le sens de erotation des moteurs
*/

#include <Arduino.h>
#include <FlexyStepper.h>
#include <StateMachine.h>
#include "Button.h"

//initialisation des sorties
#define M_grille  12 //moteur A
const int M_avancer = 2; //moteur X
const int V_gabari_entre1 = 47; 
const int V_gabari_entre2 = 48;
const int V_gabari_PWM = 46;
#define M_decoupe1  3 //moteur Y
#define M_decoupe2  4 //moteur Z
const int Led_marche = 38;
const int Led_erreur = 39;

//initialisation des mémoire pour les variables d'états des sorties
bool E_M_grille;
bool E_M_avancer;
bool E_V_gabari_entre;
bool E_V_gabari_sortie;
bool E_M_decoupe1;
bool E_M_decoupe2;
bool E_Led_marche;
bool E_Led_erreur;

//initialisation des entrées  ETAT HAUT (1) = QUAND INACTIF, ETAT BAS(0) QUAND ACTIF
const int BP_AU = 21; //pin interrupt
#define C_pos_grille1 24 //tranche
const int C_pos_grille2 = 18; //cube 
const int C_pos_grille3 = 25; //batonnet
#define C_FC_verin_entre  28
const int C_FC_verin_sorti = 26; 
#define C_FC_descenteFil_haut1  16
#define C_FC_descenteFil_haut2  17
const int C_FC_deplacementFromage_debut = 14;
const int C_FC_deplacementFromage_fin = 15;
const int BP_man = 37; 
const int BP_auto = 36; 
const int BP_tranche = 30; 
const int BP_cube = 32; 
const int BP_batonnet = 31; 
#define  BP_start  22 
const int BP_stop = 20; //pin interrup
const int BP_resetPosition = 23; 
const int BP_manu_avancer_M_avancer = 40;
const int BP_manu_reculer_M_avancer = 41;
const int C_courant = A8; //capteur de courant pour la sur intensité
const int C_porteOuverte = 19;

const int D_M_avancer = 5; //moteur X
const int D_M_grille = 13; //moteur A
const int D_M_decoupe1 = 6;//moteur y
const int D_M_decoupe2 = 7;//moteur Z

const int ENABLE = 8;

//initialisation des mémoire pour les variables d'états des entrées
bool E_C_pos_grille;
bool E_BP_AU;
bool E_C_pos_grille1; //tranche
bool E_C_pos_grille2; //cube
bool E_C_pos_grille3; //batonnet
bool E_C_FC_verin_entre;
bool E_C_FC_verin_sorti;
bool E_C_FC_descenteFil_haut1;
bool E_C_FC_descenteFil_haut2; 
bool E_C_FC_deplacementFromage_debut;
bool E_C_FC_deplacementFromage_fin;
bool E_BP_man;
bool E_BP_auto;
bool E_BP_tranche;
bool E_BP_cube;
bool E_BP_batonnet;
bool E_BP_start;
bool E_BP_stop;
bool E_BP_resetPosition;
bool E_BP_manu_avancer_M_avancer;
bool E_BP_manu_reculer_M_avancer;
bool E_C_courant;
bool E_C_porteOuverte;


//initialisation des différentes mémoire
int Etape = 0; //num étape du G7
int NbPas_M_avancer = 0;
long NbPas_M_decoupe1 = 0;
int NbPas_M_decoupe2 = 0;
int NbPas_M_grille = 0;
int nbPas_V = 0;
unsigned long prevMillis = 0;

bool tesPasseParLa = LOW;

int NbPas_M_decoupe_T = 0;
bool flagArretUrgence = LOW;
bool flagStop = LOW;
bool flagPorteOuverte = LOW;
bool Verin_OK_temp = LOW;

long Valeur_D_M_decoupe1 = -1; //1 ou -1, il s agit du sens de rotation du moteur pour la fonction d arret jusqu au fin de course
long Valeur_D_M_decoupe2 = 1;
long Valeur_D_M_avancer = 1;
long Valeur_D_M_grille = 1;


const int STATE_DELAY = 10;
const int LED = 13;
Button BP_Start_Obj(BP_start); // Connect your button between pin 2 and GND
Button C_FC_verin_entre_Obj(C_FC_verin_entre);




StateMachine machine = StateMachine();
void state0();
void state1();//remontée du vérin
void state2();//avance le fromage de 1,5 cm
void state3();
void state4();
void state5();
void state6();

// void state6();
bool transitionS0S1();
bool transitionS1S2();
bool transitionS2S3();
bool transitionS3S4();
bool transitionS4S5();
bool transitionS5S6();
bool transitionS5S2();

// bool transitionS5S6();
// bool transitionS6S0();


// bool transitionS5S2();
bool transitionS1S0();

State* S0 = machine.addState(&state0);// mode manuel
State* S1 = machine.addState(&state1);// début du mode automatique
State* S2 = machine.addState(&state2);
State* S3 = machine.addState(&state3);
State* S4 = machine.addState(&state4);
State* S5 = machine.addState(&state5);
State* S6 = machine.addState(&state6);


FlexyStepper stepper_M_decoupe1;
FlexyStepper stepper_M_decoupe2;
FlexyStepper stepper_M_avancer;
FlexyStepper stepper_M_grille;

#define homingSpeedInMMPerSec  5
#define maxHomingDistanceInMM  380   // since my lead-screw is 38cm long, should never move more than that
#define directionTowardHome  -1      // direction to move toward limit switch: 1 goes positive direction, -1 backward

#define directionTowardHome_grille 1

void arretUrgence(); //fonction arret d urgence
void stop();
void PorteOuverte();
void positionInitiale();
void trancheuse();

void setup() {
Serial.begin(9600);

    pinMode(BP_AU, INPUT_PULLUP);
    pinMode(C_pos_grille1, INPUT_PULLUP);
    pinMode(C_pos_grille2, INPUT_PULLUP);
    pinMode(C_pos_grille3, INPUT_PULLUP);
    pinMode(C_FC_verin_entre, INPUT_PULLUP);
    pinMode(C_FC_verin_sorti, INPUT_PULLUP);
    pinMode(C_FC_descenteFil_haut1, INPUT_PULLUP);
    pinMode(C_FC_descenteFil_haut2, INPUT_PULLUP);
    pinMode(C_FC_deplacementFromage_debut, INPUT_PULLUP);
    pinMode(C_FC_deplacementFromage_fin,INPUT_PULLUP);
    pinMode(BP_man, INPUT_PULLUP);
    pinMode(BP_auto, INPUT_PULLUP);
    pinMode(BP_tranche, INPUT_PULLUP);
    pinMode(BP_cube, INPUT_PULLUP);
    pinMode(BP_batonnet, INPUT_PULLUP);
    pinMode(BP_start, INPUT_PULLUP);
    pinMode(BP_stop, INPUT_PULLUP);
    pinMode(BP_resetPosition, INPUT_PULLUP);
    pinMode(BP_manu_avancer_M_avancer, INPUT_PULLUP);
    pinMode(BP_manu_reculer_M_avancer, INPUT_PULLUP);
    pinMode(C_porteOuverte, INPUT_PULLUP);
    pinMode(C_courant, INPUT_PULLUP);

    //on set les variable comme étant des OutPuts
    pinMode(M_grille, OUTPUT);
    pinMode(M_avancer, OUTPUT);
    pinMode(V_gabari_entre1, OUTPUT);
    pinMode(V_gabari_entre2, OUTPUT);
    pinMode(V_gabari_PWM, OUTPUT);
    //   pinMode(M_decoupe1, OUTPUT);
    //   pinMode(M_decoupe2, OUTPUT);
    pinMode(Led_marche, OUTPUT);
    pinMode(Led_erreur, OUTPUT);
    pinMode(ENABLE, OUTPUT);

    digitalWrite(ENABLE, LOW); //Low to enable

    stepper_M_decoupe1.connectToPins(M_decoupe1, D_M_decoupe1);
    stepper_M_decoupe2.connectToPins(M_decoupe2, D_M_decoupe2);
    stepper_M_avancer.connectToPins(M_avancer, D_M_avancer);
    stepper_M_grille.connectToPins(M_grille, D_M_grille);

    attachInterrupt(digitalPinToInterrupt(BP_AU), arretUrgence, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BP_stop), stop, CHANGE);
    attachInterrupt(digitalPinToInterrupt(C_porteOuverte), PorteOuverte, FALLING);
    flagArretUrgence = LOW;   //aru désactivé
    flagStop = LOW;           //stop désactivé
    flagPorteOuverte = LOW;   //porte fermée, cad capteur actif


    S0->addTransition(&transitionS0S1,S1);
    S1->addTransition(&transitionS1S2,S2);
    S2->addTransition(&transitionS2S3,S3);
  S3->addTransition(&transitionS3S4,S4);
  S4->addTransition(&transitionS4S5,S5);
  S5->addTransition(&transitionS5S2,S2);
  
  //S5->addTransition(&transitionS6S0,S0);
}


//=======================================
void state0(){
   if (machine.executeOnce)
  {
   Serial.println("State 0");
  //remonté du vérin
  }
    

    
}
bool transitionS0S1(){
  // if (digitalRead(BP_start)==0)
  // {
  //   Serial.println("state0 =>state1");/* code */
  // }
  
  if (BP_Start_Obj.released()){
    Serial.println("state0 =>state1");
    return true;
  }   
}

void state1(){
  //Serial.println("State 0");
  //remonté du vérin
  if (machine.executeOnce)
  {
    digitalWrite(Led_erreur, LOW);
    digitalWrite(Led_marche,HIGH);

        //force l utilisateur a valider son choix au DEBUT du programme UNIQUEMENT (ne sera plus lu après)
        E_BP_tranche = digitalRead(BP_tranche);
        E_BP_batonnet = digitalRead(BP_batonnet);
        E_BP_cube = digitalRead(BP_cube);
//remontée du vérin
        digitalWrite(V_gabari_entre1, LOW);
        digitalWrite(V_gabari_entre2, HIGH);
        analogWrite(V_gabari_PWM, 255);
        stepper_M_decoupe1.setSpeedInStepsPerSecond(100);        
        stepper_M_decoupe1.setAccelerationInStepsPerSecondPerSecond(100);
        stepper_M_decoupe1.setTargetPositionInSteps(-20);

        stepper_M_decoupe2.setSpeedInStepsPerSecond(100);        
        stepper_M_decoupe2.setAccelerationInStepsPerSecondPerSecond(100);
        stepper_M_decoupe2.setTargetPositionInSteps(-20);

        stepper_M_grille.setSpeedInStepsPerSecond(100);
        stepper_M_grille.setAccelerationInStepsPerSecondPerSecond(100);
        stepper_M_grille.setTargetPositionInSteps(-20);

  }

  if (C_FC_verin_entre_Obj.pressed())
  {
    analogWrite(V_gabari_PWM, 0);/* Arret du moteur */
  }
  
  if (!stepper_M_decoupe1.motionComplete())
  {
    stepper_M_decoupe1.processMovement();

  }
  
  stepper_M_decoupe2.processMovement();
  stepper_M_grille.processMovement();
  
  if (digitalRead(C_FC_descenteFil_haut1) == HIGH) 
    {     
      
      stepper_M_decoupe1.setTargetPositionToStop();
      E_C_FC_descenteFil_haut1=true;
      
    }

    if (digitalRead(C_FC_descenteFil_haut2) == HIGH) 
    {     
      
      stepper_M_decoupe2.setTargetPositionToStop();
      E_C_FC_descenteFil_haut1=true;
      
    }
    if (digitalRead(C_pos_grille1) == HIGH) 
    {     
      
      stepper_M_grille.setTargetPositionToStop();
      E_C_pos_grille1=true;
      
    }
    
           
       // positionInitiale();
        Serial.println("position initiale atteinte");
       // Etape = 1;
   // BP_start

    
}
bool transitionS1S2(){
  
  //  if (button1.released()){
  //   Serial.println("state0 =>state1");
    return E_C_FC_descenteFil_haut1||E_C_FC_descenteFil_haut2||E_C_pos_grille1;
  // }   
}

// void state1(){
  
//   //remonté du vérin
//   if (machine.executeOnce)
//   {
//     Serial.println("State 1");
//     stepper_M_decoupe1.setStepsPerMillimeter(25 * 1);    // 1x microstepping
//     stepper_M_decoupe1.setSpeedInMillimetersPerSecond(10.0);
//     stepper_M_decoupe1.setAccelerationInMillimetersPerSecondPerSecond(10.0); 
    
//     stepper_M_decoupe2.setStepsPerMillimeter(25 * 1);    // 1x microstepping
//     stepper_M_decoupe2.setSpeedInMillimetersPerSecond(10.0);
//     stepper_M_decoupe2.setAccelerationInMillimetersPerSecondPerSecond(10.0);

//     stepper_M_grille.setStepsPerMillimeter(25 * 1);    // 1x microstepping
//     stepper_M_grille.setSpeedInMillimetersPerSecond(10.0);
//     stepper_M_grille.setAccelerationInMillimetersPerSecondPerSecond(10.0);

//     stepper_M_decoupe1.moveToHomeInMillimeters(directionTowardHome, homingSpeedInMMPerSec, 
//                                     maxHomingDistanceInMM, C_FC_descenteFil_haut1);
//   stepper_M_decoupe2.moveToHomeInMillimeters(directionTowardHome, homingSpeedInMMPerSec, 
//                                     maxHomingDistanceInMM, C_FC_descenteFil_haut2);
//   stepper_M_grille.moveToHomeInMillimeters(directionTowardHome, homingSpeedInMMPerSec, 
//                                     maxHomingDistanceInMM, C_pos_grille1);
//   //remontée du vérin
//   digitalWrite(V_gabari_entre1, LOW);
//   digitalWrite(V_gabari_entre2, HIGH);
//   analogWrite(V_gabari_PWM, 255);
//   Serial.println("state1()");

//   }
//   if (C_FC_verin_entre_Obj.pressed())
//   {
//     analogWrite(V_gabari_PWM, 0);/* Arret du moteur */
//     Serial.println("Gabarit remonté");
//   }
  

    
// }
// bool transitionS1S2(){
  

//   if(BP_Start_Obj.released())    
//    {
//     Serial.println("state1 =>state2");
//     return true;    
//     }   
// }

void state2()
{
if (machine.executeOnce)  
{
 Serial.println("State 2");
 stepper_M_avancer.setTargetPositionInSteps(3000) ;
}
stepper_M_avancer.processMovement();
}


bool transitionS2S3(){
  
  if (stepper_M_avancer.motionComplete())
  {
    return true;/* code */
  }
  
  //  if (button1.released()){
  //   Serial.println("state0 =>state1");
    //return E_C_FC_descenteFil_haut1||E_C_FC_descenteFil_haut2||E_C_pos_grille1;
  // }   
}

void state3()
{
if (machine.executeOnce)  
{
 stepper_M_avancer.setTargetPositionInSteps(3000) ;// avec fromage 15mm
}
stepper_M_avancer.processMovement();

}


bool transitionS3S4(){
  
  if (stepper_M_avancer.motionComplete())
  {
    return true;/* code */
  }
  
  //  if (button1.released()){
  //   Serial.println("state0 =>state1");
    //return E_C_FC_descenteFil_haut1||E_C_FC_descenteFil_haut2||E_C_pos_grille1;
  // }   
}

void state4()
{
if (machine.executeOnce)  
{
 stepper_M_avancer.setTargetPositionInSteps(3000) ;
}
stepper_M_avancer.processMovement();


}


bool transitionS4S5(){
  
  if (stepper_M_avancer.motionComplete())
  {
    return true;/* code */
  }
  
  //  if (button1.released()){
  //   Serial.println("state0 =>state1");
    //return E_C_FC_descenteFil_haut1||E_C_FC_descenteFil_haut2||E_C_pos_grille1;
  // }   
}

void state5()
{
if (machine.executeOnce)  
{
 stepper_M_decoupe1.setTargetPositionInSteps(34800);
 stepper_M_decoupe2.setTargetPositionInSteps(34800);

}
stepper_M_decoupe1.processMovement();
stepper_M_decoupe2.processMovement();

}

bool transitionS5S6(){
  
  return stepper_M_decoupe1.motionComplete() && stepper_M_decoupe2.motionComplete();
} 

void state6()//remontée gabari et fil
{
if (machine.executeOnce)  
{
 stepper_M_decoupe1.setTargetPositionInSteps(10);//remonté fil
 stepper_M_decoupe2.setTargetPositionInSteps(10);
 digitalWrite(V_gabari_entre1, HIGH);// descente du gabarit
 digitalWrite(V_gabari_entre2, LOW);
 analogWrite(V_gabari_PWM, 255);

}
stepper_M_decoupe1.processMovement();
stepper_M_decoupe2.processMovement();

}

bool transitionS5S2(){
  
  return stepper_M_decoupe1.motionComplete() && stepper_M_decoupe2.motionComplete();
} 



void loop() {
machine.run();

  //partie calibration
  E_BP_manu_avancer_M_avancer = digitalRead(BP_manu_avancer_M_avancer);
  E_BP_manu_reculer_M_avancer = digitalRead(BP_manu_reculer_M_avancer);

  //mode manu
  while(digitalRead(BP_man) == 0 && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW) //read bp man est il utile?
  {
      while(digitalRead(BP_manu_reculer_M_avancer) == LOW && digitalRead(C_FC_deplacementFromage_debut) == 0)
      {
        {
          digitalWrite(D_M_avancer, HIGH);
          digitalWrite(M_avancer, HIGH);
          delay(1); // microstepping activé
          digitalWrite(M_avancer, LOW);
          delay(1); 
        }
      }
      while(digitalRead(BP_manu_avancer_M_avancer) == LOW && digitalRead(C_FC_deplacementFromage_fin) == 0)
      {
        {
          digitalWrite(D_M_avancer, LOW);
          digitalWrite(M_avancer, HIGH);
          delay(1);
          digitalWrite(M_avancer, LOW);
          delay(1); 
        }
      }
  }
  
//   //mode auto
//   while(flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW && flagPorteOuverte == LOW && digitalRead(BP_auto) == 0)
//   {
//     //mise en marche et remise en CI
//     if(Etape == 0 && digitalRead(BP_auto)== 0 && digitalRead(BP_start)== 0) //remise en CI
//     {
//         Serial.print("Num etape: ");
//         Serial.println(Etape);

//         digitalWrite(Led_erreur, LOW);
//         digitalWrite(Led_marche,HIGH);

//         //force l utilisateur a valider son choix au DEBUT du programme UNIQUEMENT (ne sera plus lu après)
//         E_BP_tranche = digitalRead(BP_tranche);
//         E_BP_batonnet = digitalRead(BP_batonnet);
//         E_BP_cube = digitalRead(BP_cube);
    
//         positionInitiale();
//         Serial.println("position initiale atteinte");
//         Etape = 1;
//     }

//     E_C_FC_descenteFil_haut1 = digitalRead(C_FC_descenteFil_haut1);
//     E_C_FC_descenteFil_haut2 = digitalRead(C_FC_descenteFil_haut2);
//     E_C_FC_verin_entre = digitalRead(C_FC_verin_entre);
//     E_BP_start = digitalRead(BP_start);
//     E_BP_auto = digitalRead(BP_auto);

//     //avance le fromage de 1,5 cm
//     if(Etape == 1 && E_C_FC_descenteFil_haut1 == 1 && E_C_FC_descenteFil_haut2 == 1 && E_C_FC_verin_entre == 1 && E_BP_auto == 0)
//     {
//         Serial.print("Num etape: ");
//         Serial.println(Etape);
      
//       //remettre a 3000
//       while(NbPas_M_avancer < 3000 && digitalRead(C_FC_deplacementFromage_fin) == 0 && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW) //200*7,5 note: il saute des pas, on est a 1,35 cm et pas 1,5
//       {
//         digitalWrite(D_M_avancer, LOW); //avance
//         digitalWrite(M_avancer, HIGH);
//         delay(1);
//         digitalWrite(M_avancer, LOW);
//         delay(1); 
//         NbPas_M_avancer++;
//       }
//       Etape = 2;
//     }

//     E_C_pos_grille1 = digitalRead(C_pos_grille1);
//     E_C_pos_grille2 = digitalRead(C_pos_grille2);
//     E_C_pos_grille3 = digitalRead(C_pos_grille3);
    
//     // déplacement grille par rapport au choix
//     if(Etape == 2 && E_BP_tranche == 0 && digitalRead(C_pos_grille1) == 1) //tranche
//     {
//       Serial.print("Num etape: ");
//       Serial.println(Etape);

//       trancheuse();

//       Etape = 3;
//     }
//     if(Etape == 2 && E_BP_cube == 0 && digitalRead(C_pos_grille1) == 1) //cube
//     {
//       Serial.print("Num etape: ");
//       Serial.println(Etape);
      
//       while(digitalRead(C_pos_grille2) == 0 && flagArretUrgence == LOW && flagStop == LOW)
//       {
//         digitalWrite(D_M_grille, HIGH); //avance vers l opposé du coffret electrique
//         digitalWrite(M_grille, HIGH);
//         delay(1);
//         digitalWrite(M_grille, LOW);
//         delay(1); 
//       }

//       trancheuse();
//       Etape = 4;
//     }
//     if(Etape == 2 && E_BP_batonnet == 0 && digitalRead(C_pos_grille1) == 1) //batonnet
//     {
//       Serial.print("Num etape: ");
//       Serial.println(Etape);
//       while(digitalRead(C_pos_grille3) == 0 && flagArretUrgence == LOW && flagStop == LOW)
//       {

//         digitalWrite(D_M_grille, HIGH); //avance vers l opposé du coffret electrique
//         digitalWrite(M_grille, HIGH);
//         delay(1);
//         digitalWrite(M_grille, LOW);
//         delay(1);
//       } 

//     trancheuse();
//     Etape = 5;
//     }

  
//     //alignement de la grille avec le gabari en mode cube
//     if(Etape == 4 && digitalRead(C_FC_descenteFil_haut1) == 1 && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW)
//     {
//       Serial.print("Num etape: ");
//       Serial.println(Etape);
//       while(NbPas_M_grille < 1200)
//       {
//         digitalWrite(D_M_grille, LOW); //recule (vers ca pos initiale)
//         digitalWrite(M_grille, HIGH);
//         delay(3);
//         digitalWrite(M_grille, LOW);
//         delay(3); 
//         NbPas_M_grille++;
//       }
//       Etape = 6;
//     }
//     //alignement de la grille avec le gabari en mode tranche
//     if(Etape == 5 && digitalRead(C_FC_descenteFil_haut1) == 1 && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW)
//     {
//       Serial.print("Num etape: ");
//       Serial.println(Etape);
//       while(NbPas_M_grille < 2300)
//       {
//         digitalWrite(D_M_grille, LOW); //recule (vers ca pos initiale)
//         digitalWrite(M_grille, HIGH);
//         delay(3);
//         digitalWrite(M_grille, LOW);
//         delay(3); 
//         NbPas_M_grille++;
//       }
//       Etape = 6;
//     }

//     //descente du gabari
//     if(Etape == 6 && NbPas_M_grille >= 1200)
//     {
//       Serial.print("Num etape: ");
//       Serial.println(Etape);
      
//       while(digitalRead(C_FC_verin_sorti) == 0 && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW)
//       {
//         digitalWrite(V_gabari_entre1, HIGH);
//         digitalWrite(V_gabari_entre2, LOW);
//         analogWrite(V_gabari_PWM, 255);
//       }
//       digitalWrite(V_gabari_entre1, LOW);
//       digitalWrite(V_gabari_entre2, LOW);

//       Etape = 7;
//     }

//     //remontée du gabari
//     if(Etape == 7 && digitalRead(C_FC_verin_sorti) == 1)
//     {
//       Serial.print("Num etape: ");
//       Serial.println(Etape);
//       while(digitalRead(C_FC_verin_entre) == 0 && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW)
//       {
//         digitalWrite(V_gabari_entre1, LOW);
//         digitalWrite(V_gabari_entre2, HIGH);
//         analogWrite(V_gabari_PWM, 255);
//       }

//       digitalWrite(V_gabari_entre1, LOW);
//       digitalWrite(V_gabari_entre2, LOW);

//       NbPas_M_avancer = 0;
//       NbPas_M_decoupe1 = 0;
//       NbPas_M_decoupe2 = 0;
//       NbPas_M_grille = 0;
//       nbPas_V = 0;
//       prevMillis = 0;
//       NbPas_M_decoupe_T = 0;

//       positionInitiale();
//       digitalWrite(Led_marche, LOW);
//       Etape = 0;
//     }
//   }

//   //fonction arret d'urgence
//   if(flagArretUrgence == HIGH)
//   {
//     if(tesPasseParLa == LOW)
//     {
//       NbPas_M_decoupe1 = 0;
//     }
    

//     //arret led marche et activation de la led erreur
//     digitalWrite(Led_marche, LOW);
//     digitalWrite(Led_erreur, HIGH);
//     flagStop = LOW;

//     //remontée du gabari de 4,5 cm
//     if(digitalRead(C_FC_verin_entre) == 0 && Verin_OK_temp == LOW) //si le vérin n'est pas déja en haut
//     {
//       Serial.println("Remonté du vérin en cours");
//       prevMillis = millis();
      

//       while( millis() < prevMillis +3000)//pendant 3 sec, vt max est de 1,5 cm/s //rajouter le truc ede merde avec le machin qui mon en fonction du pas relou
//       {
//         digitalWrite(V_gabari_entre1, LOW);
//         digitalWrite(V_gabari_entre2, HIGH);
//         analogWrite(V_gabari_PWM, 255);
//       }
//       digitalWrite(V_gabari_entre1, LOW);
//       digitalWrite(V_gabari_entre2, LOW);
//       Verin_OK_temp = HIGH;
//       Serial.println("t es sorti");
//     }

//     //IL Y A UN PB ICI
//     //remontée de la trancheuse 
//     Serial.println("La remontée de la trancheuse va débuter");
    
//     while(NbPas_M_decoupe1 < 10000  && (digitalRead(C_FC_descenteFil_haut1) == 0 && digitalRead(C_FC_descenteFil_haut2) == 0) && tesPasseParLa == LOW) //5cm de hauteur
//     {
//       digitalWrite(D_M_decoupe1, HIGH); //monte
//       digitalWrite(D_M_decoupe2, HIGH);
//       digitalWrite(M_decoupe1, HIGH);
//       digitalWrite(M_decoupe2, HIGH);
//       delayMicroseconds(571);
//       digitalWrite(M_decoupe1, LOW);
//       digitalWrite(M_decoupe2, LOW);
//       delayMicroseconds(571);
//       NbPas_M_decoupe1++;
//     }
//   Serial.println("la trancheuse est remontée");
//   tesPasseParLa = HIGH;

//     if(digitalRead(BP_resetPosition) == 0 && digitalRead(BP_AU) == 0)
//     {
//         Serial.println("tu as appuyé sur le bp reset position");
//         flagArretUrgence = LOW;
//         positionInitiale();
//         Etape = 0;
//     }
//   }
  
//   //fonction stop
//   if(flagStop == HIGH)
//   {

//     flagArretUrgence = LOW;

//     //arret led marche et activation de la led erreur
//     digitalWrite(Led_marche, LOW);
//     digitalWrite(Led_erreur, HIGH);

//     //arret de toutes les sorties
//     digitalWrite(V_gabari_entre1, LOW);
//     digitalWrite(V_gabari_entre2, LOW);

//     digitalWrite(M_decoupe1, HIGH); 
//     digitalWrite(M_decoupe2, HIGH);
//     digitalWrite(M_avancer, HIGH);
//     digitalWrite(M_grille, HIGH);

//     if(digitalRead(BP_resetPosition) == 0) 
//     {
//         Serial.println("tu as appuyé sur le bp reset position");
//         flagStop = LOW;
//         positionInitiale();
//         Etape = 0;
        
//     }
//   }
  
//   //fonction porte ouverte
//   while(flagPorteOuverte == HIGH) //porte ouverte
//   {

//     digitalWrite(Led_marche, LOW);
//     digitalWrite(Led_erreur, HIGH);

//     //arret de toutes les sorties
//     digitalWrite(V_gabari_entre1, LOW);
//     digitalWrite(V_gabari_entre2, LOW);

//     digitalWrite(M_decoupe1, HIGH); 
//     digitalWrite(M_decoupe2, HIGH);
//     digitalWrite(M_avancer, HIGH);
//     digitalWrite(M_grille, HIGH);

//     if(digitalRead(BP_resetPosition) == 0 && digitalRead(C_porteOuverte) == 1) 
//     {
//         Serial.println("tu as appuyé sur le bp reset position");
//         flagPorteOuverte = LOW;    
//         positionInitiale();
//         Etape = 0;    
//     }
//   }

 }

void arretUrgence()
{
  Verin_OK_temp = LOW;
  Serial.println("aru activé ");
  flagArretUrgence = HIGH;
}

void stop()
{
  Serial.println("stop activé ");
  flagStop = HIGH;
}

void PorteOuverte()
{
  Serial.println("Porte ouverte ");
  flagPorteOuverte = HIGH;
}

void positionInitiale()
{
    digitalWrite(Led_erreur, LOW);
    
    //remontée du vérin
    while(digitalRead(C_FC_verin_entre) == 0 && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW)
    {
        digitalWrite(V_gabari_entre1, LOW);
        digitalWrite(V_gabari_entre2, HIGH);
        analogWrite(V_gabari_PWM, 255);
    }
    Serial.println("Vérin monté");

    //remontée du fil
    E_C_FC_descenteFil_haut1 = digitalRead(C_FC_descenteFil_haut1);
    E_C_FC_descenteFil_haut2 = digitalRead(C_FC_descenteFil_haut2);
    
    
    while((digitalRead(C_FC_descenteFil_haut1) == 0 || digitalRead(C_FC_descenteFil_haut2) == 0) && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW)
    {
      Serial.println("trancheuse activée");
      
        while(digitalRead(C_FC_descenteFil_haut1) == 0 && digitalRead(C_FC_descenteFil_haut2) == 1 && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW)
        {
          
            digitalWrite(D_M_decoupe1, HIGH); // monter
            digitalWrite(M_decoupe1, HIGH);
            digitalWrite(M_decoupe2, HIGH);
            delayMicroseconds(571);
            digitalWrite(M_decoupe1, LOW);
            delayMicroseconds(571); 
        }
        while(digitalRead(C_FC_descenteFil_haut1) == 1 && digitalRead(C_FC_descenteFil_haut2) == 0 && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW)
        {
            digitalWrite(D_M_decoupe2, HIGH);
            digitalWrite(M_decoupe1, HIGH);
            digitalWrite(M_decoupe2, HIGH);
            delayMicroseconds(571);
            digitalWrite(M_decoupe2, LOW);
            delayMicroseconds(571);
        }
        while(digitalRead(C_FC_descenteFil_haut1) == 0 && digitalRead(C_FC_descenteFil_haut2) == 0 && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW)
        {
            digitalWrite(D_M_decoupe1, HIGH);
            digitalWrite(D_M_decoupe2, HIGH);
            digitalWrite(M_decoupe1, HIGH);
            digitalWrite(M_decoupe2, HIGH);
            delayMicroseconds(571);
            digitalWrite(M_decoupe1, LOW);
            digitalWrite(M_decoupe2, LOW);
            delayMicroseconds(571);
        }
    }
    Serial.println("trancheuse coupé");
    Serial.println("grille activée");
    //retour a la position initiale de la grille
    while(digitalRead(C_pos_grille1) == 0 && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW) 
    {
      
      digitalWrite(D_M_grille, LOW); //recule
      digitalWrite(M_grille, HIGH);
      delay(1);
      digitalWrite(M_grille, LOW);
      delay(1); 
    }
    Serial.println("Grille rentrée");
}

void trancheuse()
{
  //descente de la trancheuse
    if((((digitalRead(C_pos_grille3) && E_BP_batonnet == 0) || (E_BP_cube == 0 && digitalRead(C_pos_grille2)) || (E_BP_tranche == 0 && digitalRead(C_pos_grille1)))) && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW)
    {
      Serial.print("Num etape: ");
      Serial.println(Etape);
      //8700 * 34 800
      while(NbPas_M_decoupe1 < 34800 && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW) //200*8cm les 200 en plus sont pour desendre le fil au plus bas
      {
        digitalWrite(D_M_decoupe1, LOW); //descend
        digitalWrite(D_M_decoupe2, LOW);
        digitalWrite(M_decoupe1, HIGH);
        digitalWrite(M_decoupe2, HIGH);
        delayMicroseconds(571);
        digitalWrite(M_decoupe1, LOW);
        digitalWrite(M_decoupe2, LOW);
        delayMicroseconds(571); 
        NbPas_M_decoupe1++;
      }
    }

    //remontée de la trancheuse
    if( NbPas_M_decoupe1 >= 34800 && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW) 
    {
      Serial.print("Num etape: ");
      Serial.println(Etape);
      while(digitalRead(C_FC_descenteFil_haut1) == 0)
      {
        digitalWrite(D_M_decoupe1, HIGH); //monte
        digitalWrite(D_M_decoupe2, HIGH);
        digitalWrite(M_decoupe1, HIGH);
        digitalWrite(M_decoupe2, HIGH);
        delayMicroseconds(571);
        digitalWrite(M_decoupe1, LOW);
        digitalWrite(M_decoupe2, LOW);
        delayMicroseconds(571);
      }
    }

}

//MOTEUR 1 ET 2 SACAD2 L UN APRES L AUTRE
// if(BP_start == 1)
// {
//   long nbStep = 200;
//   stepper_M_decoupe1.setSpeedInStepsPerSecond(200);// nb step/sec
//   stepper_M_decoupe1.setAccelerationInStepsPerSecondPerSecond(1600); 
//   stepper_M_decoupe1.moveRelativeInSteps(nbStep); //200 step par 2 mm
//   stepper_M_decoupe2.setSpeedInStepsPerSecond(200);
//   stepper_M_decoupe2.setAccelerationInStepsPerSecondPerSecond(1600); 
//   stepper_M_decoupe2.moveRelativeInSteps(nbStep);
// }

//AVANCE JUSQU AU FIN DE COURSE
// stepper_M_decoupe1.moveToHomeInRevolutions(Valeur_D_M_decoupe1, 10, 1000, BP_start);
// stepper_M_decoupe2.moveToHomeInRevolutions(Valeur_D_M_decoupe1, 10, 1000, BP_resetPosition);

//SI FC PAS ACTIVE, MOTEUR 1 ET 2 FONCTIONNE EN MEME TEMPS
//   while(E_BP_stop == 1 && E_BP_resetPosition == 1)
//   {
//   digitalWrite(D_M_decoupe1, HIGH);
//   digitalWrite(D_M_decoupe2, HIGH);
//   digitalWrite(M_decoupe1, HIGH);
//   digitalWrite(M_decoupe2, HIGH);
//   delay(1);
//   digitalWrite(M_decoupe1, LOW);
//   digitalWrite(M_decoupe2, LOW);
//   delay(1); 
//   E_BP_stop = digitalRead(BP_stop);
//   E_BP_resetPosition = digitalRead(BP_resetPosition);
//   }

//MOTEUR AVANCE FAST
//  E_BP_start = digitalRead(BP_start);
//   E_BP_resetPosition = digitalRead(BP_resetPosition);
//   if(E_BP_start == 0)
//   { 
//     digitalWrite(Led_marche,HIGH);
//    long nbStep = -20000; //- reculer +avancé
//   stepper_M_avancer.setSpeedInStepsPerSecond(400);// nb step/sec
//   stepper_M_avancer.setAccelerationInStepsPerSecondPerSecond(1600); 
//   stepper_M_avancer.moveRelativeInSteps(nbStep); //200 step par 2 mm
//   }

// GESTION VERIN
// E_BP_stop = digitalRead(BP_stop);
//   E_BP_resetPosition = digitalRead(BP_resetPosition);
  
//   while(E_BP_stop == 1 && E_BP_resetPosition == 1)
//   {
//     //DESCENTE DU VERIN
 
//       digitalWrite(V_gabari_entre1, LOW);
//       digitalWrite(V_gabari_entre2, LOW);
//       delay(100);
//       digitalWrite(V_gabari_entre1, HIGH);
//       digitalWrite(V_gabari_entre2, LOW);
//       analogWrite(V_gabari_PWM, 255);
//       delay(7000);
//       E_BP_stop = digitalRead(BP_stop);
//       E_BP_resetPosition = digitalRead(BP_resetPosition);

//     E_BP_stop = digitalRead(BP_stop);
//     E_BP_resetPosition = digitalRead(BP_resetPosition);
//     //MONTER VERIN
//   if(millis() - prevMillis > 15000 && E_BP_stop == 1 && E_BP_resetPosition == 1)
//     {
//       digitalWrite(V_gabari_entre1, LOW);
//       digitalWrite(V_gabari_entre2, LOW);
//       delay(100);
//       digitalWrite(V_gabari_entre1, LOW);
//       digitalWrite(V_gabari_entre2, HIGH);
//       analogWrite(V_gabari_PWM, 255);
//       prevMillis = millis();

//     }
//     E_BP_stop = digitalRead(BP_stop);
//     E_BP_resetPosition = digitalRead(BP_resetPosition);

//   }

  //MOTEUR AVANCE FAST
//  E_BP_start = digitalRead(BP_start);
//   E_BP_resetPosition = digitalRead(BP_resetPosition);
//   if(E_BP_start == 0)
//   { 
//     digitalWrite(Led_marche,HIGH);
//    long nbStep = -20000; //- reculer +avancé
//   stepper_M_avancer.setSpeedInStepsPerSecond(400);// nb step/sec
//   stepper_M_avancer.setAccelerationInStepsPerSecondPerSecond(1600); 
//   stepper_M_avancer.moveRelativeInSteps(nbStep); //200 step par 2 mm
//   }

// while(NbPas_M_avancer < 1500) //200*7,5
//     {
//       digitalWrite(D_M_decoupe1, HIGH); // A VERIFIER SI IL MONTE OU IL DESCEND
//       digitalWrite(D_M_decoupe2, HIGH);
//       digitalWrite(M_decoupe1, HIGH);
//       digitalWrite(M_decoupe2, HIGH);
//       delay(1);
//       digitalWrite(M_decoupe1, LOW);
//       digitalWrite(M_decoupe2, LOW);
//       delay(1); 
//       NbPas_M_avancer++;
//     }

  // if(Etape == 3 && ((digitalRead(E_C_pos_grille3) == 0 && E_BP_batonnet == 0)||(digitalRead(E_C_pos_grille2) == 0 && E_BP_cube == 0)||(digitalRead(E_C_pos_grille1) == 0 && E_BP_tranche == 0)) )
  // {
  //   while()

  // }