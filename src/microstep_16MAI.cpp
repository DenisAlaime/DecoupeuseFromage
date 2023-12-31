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
#include <KeyDetector.h>
#include <L298N.h>
#include <neotimer.h>

// Déclaration des sorties
#define M_grille 12 // moteur A
#define M_avancer 2 // moteur X
#define V_gabari_entre1 47
#define V_gabari_entre2 48
#define V_gabari_PWM 46
#define M_decoupe1 3 // moteur Y
#define M_decoupe2 4 // moteur Z
#define Led_marche 38
#define Led_erreur 39

// initialisation des mémoire pour les variables d'états des sorties
bool E_M_grille;
bool E_M_avancer;
bool E_V_gabari_entre;
bool E_V_gabari_sortie;
bool E_M_decoupe1;
bool E_M_decoupe2;
bool E_Led_marche;
bool E_Led_erreur;

// initialisation des entrées  ETAT HAUT (1) = QUAND INACTIF, ETAT BAS(0) QUAND ACTIF

#define C_pos_grille1 24 // tranche
#define C_pos_grille2 18 // cube
#define C_pos_grille3 25 // batonnet
#define C_FC_verin_entre 28
#define C_FC_verin_sorti 26
#define C_FC_descenteFil_haut1 16
#define C_FC_descenteFil_haut2 17
#define C_FC_deplacementFromage_debut 14
#define C_FC_deplacementFromage_fin 15

#define KEY_C_pos_grille1 1  // tranche
#define KEY_CC_pos_grille2 2 // cube
#define KEY_CC_pos_grille3 3 // batonnet
#define KEY_CC_FC_verin_entre 4
#define KEY_CC_FC_verin_sorti 5
#define KEY_CC_FC_descenteFil_haut1 6
#define KEY_CC_FC_descenteFil_haut2 7
#define KEY_CC_FC_deplacementFromage_debut 8
#define KEY_CC_FC_deplacementFromage_fin 9

#define Int_man 37
#define Int_auto 36
#define Int_tranche 30
#define Int_cube 32
#define Int_batonnet 31

#define BP_AU 21 // pin interrupt
#define BP_start 22
#define BP_stop 20 // pin interrup
#define BP_resetPosition 23
#define BP_manu_avancer_M 40
#define BP_manu_reculer_M 41
#define C_courant A8 // capteur de courant pour la sur intensité
#define C_porteOuverte 19

#define D_M_avancer 5  // moteur X
#define D_M_grille 13  // moteur A
#define D_M_decoupe1 6 // moteur y
#define D_M_decoupe2 7 // moteur Z

#define ENABLE 8

// initialisation des mémoire pour les variables d'états des entrées
bool E_C_pos_grille;
bool E_BP_AU;
bool E_C_pos_grille1; // tranche
bool E_C_pos_grille2; // cube
bool E_C_pos_grille3; // batonnet
bool E_C_FC_verin_entre;
bool E_C_FC_verin_sorti;
bool E_C_FC_descenteFil_haut1;
bool E_C_FC_descenteFil_haut2;
bool E_C_FC_deplacementFromage_debut;
bool E_C_FC_deplacementFromage_fin;
bool E_Int_man;
bool E_Int_auto;
bool E_Int_tranche;
bool E_Int_cube;
bool E_Int_batonnet;
bool E_BP_start;
bool E_BP_stop;
bool E_BP_resetPosition;
bool E_BP_manu_avancer_M_avancer;
bool E_BP_manu_reculer_M_avancer;
bool E_C_courant;
bool E_C_porteOuverte;

// initialisation des différentes mémoire
int Etape = 0; // num étape du G7
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

long Valeur_D_M_decoupe1 = -1; // 1 ou -1, il s agit du sens de rotation du moteur pour la fonction d arret jusqu au fin de course
long Valeur_D_M_decoupe2 = 1;
long Valeur_D_M_avancer = 1;
long Valeur_D_M_grille = 1;

const int STATE_DELAY = 10;
const int LED = 13;
Button BP_Start_Obj(BP_start);         // Connect your button between pin 2 and GND
Button BP_Reset_Obj(BP_resetPosition); //
Button BP_Stop_Obj(BP_stop);           //

Button BP_manu_avancer_M_Obj(BP_manu_avancer_M); //
Button BP_manu_reculer_M_Obj(BP_manu_reculer_M); //

Button Int_Manu_Obj(Int_man);  //
Button Int_Auto_Obj(Int_auto); //

Button Int_Tranches_Obj(Int_tranche);   //
Button Int_Batonnets_Obj(Int_batonnet); //
Button Int_Cubes_Obj(Int_cube);         //

Button C_pos_grille1_Obj(C_pos_grille1);
Button C_pos_grille2_Obj(C_pos_grille2);
Button C_pos_grille3_Obj(C_pos_grille3);

Button C_FC_verin_entre_Obj(C_FC_verin_entre);
Button C_FC_verin_sorti_Obj(C_FC_verin_sorti);

Button C_FC_descenteFil_haut1_Obj(C_FC_descenteFil_haut1);
Button C_FC_descenteFil_haut2_Obj(C_FC_descenteFil_haut2);

Button C_FC_deplacementFromage_debut_Obj(C_FC_deplacementFromage_debut);
Button C_FC_deplacementFromage_fin_Obj(C_FC_deplacementFromage_fin);

FlexyStepper stepper_M_decoupe1;
FlexyStepper stepper_M_decoupe2;
FlexyStepper stepper_M_Fromage;
FlexyStepper stepper_M_grille;

L298N GabariMotor(V_gabari_PWM, V_gabari_entre1, V_gabari_entre2);
Neotimer mytimer = Neotimer();

// #define KEY_C_pos_grille1 1  // tranche
// #define KEY_CC_pos_grille2 2 // cube
// #define KEY_CC_pos_grille3 3 // batonnet
// #define KEY_CC_FC_verin_entre 4
// #define KEY_CC_FC_verin_sorti 5
// #define KEY_CC_FC_descenteFil_haut1 6
// #define KEY_CC_FC_descenteFil_haut2 7
// #define KEY_CC_FC_deplacementFromage_debut 8
// #define KEY_CC_FC_deplacementFromage_fin 9

// Key keys[] = {{KEY_C_pos_grille1, C_pos_grille1},
//               {KEY_CC_pos_grille2, C_pos_grille2},
//               {KEY_CC_pos_grille3, C_pos_grille3},
//               {KEY_CC_FC_verin_entre, C_FC_verin_entre},
//               {KEY_CC_FC_verin_sorti, C_FC_verin_sorti},
//               {KEY_CC_FC_descenteFil_haut1, C_FC_descenteFil_haut1},
//               {KEY_CC_FC_descenteFil_haut2, C_FC_descenteFil_haut2},
//               {KEY_CC_FC_deplacementFromage_debut, C_FC_deplacementFromage_debut},
//               {KEY_CC_FC_deplacementFromage_fin, C_FC_deplacementFromage_fin}};

// Create KeyDetector object
// KeyDetector FinDeCoursesDetector(keys, sizeof(keys)/sizeof(Key));

StateMachine machine = StateMachine();
void state0();
void state1_Homing();   // remontée du vérin
void state2_ModeManu(); // avance le fromage de 1,5 cm
void state3_ModeAuto();
void state4_PositionTranches();
void state5_PositionCubes();
void state6_PositionBatonnet();
void state7_Attente();
void state8_AvancementFromage();
void state9_DescendreTranchause();
void state10_RemontonterTranchauseDescendreGabary();
void state11_RemontonterGabary();

// void state6();
bool transitionS0S1();
bool transitionS1S2();
bool transitionS1S3();
bool transitionS2S3();
bool transitionS3S4();
bool transitionS3S5();
bool transitionS3S6();

bool transitionS4S7();
bool transitionS5S7();
bool transitionS6S7();

bool transitionS7S8();
bool transitionS8S9();
bool transitionS9S10();
bool transitionS10S11();

bool transitionS11S3();

State *S0 = machine.addState(&state0);        //
State *S1 = machine.addState(&state1_Homing); //
State *S2 = machine.addState(&state2_ModeManu);
State *S3 = machine.addState(&state3_ModeAuto);
State *S4 = machine.addState(&state4_PositionTranches);
State *S5 = machine.addState(&state5_PositionCubes);
State *S6 = machine.addState(&state6_PositionBatonnet);
State *S7 = machine.addState(&state7_Attente); //
State *S8 = machine.addState(&state8_AvancementFromage);
State *S9 = machine.addState(&state9_DescendreTranchause);
State *S10 = machine.addState(&state10_RemontonterTranchauseDescendreGabary);
State *S11 = machine.addState(&state11_RemontonterGabary);

#define homingSpeedInMMPerSec 5
#define maxHomingDistanceInMM 380 // since my lead-screw is 38cm long, should never move more than that
#define directionTowardHome -1    // direction to move toward limit switch: 1 goes positive direction, -1 backward

#define directionTowardHome_grille 1

void arretUrgence(); // fonction arret d urgence
void stop();
void PorteOuverte();
void positionInitiale();
bool trancheuse();
bool DeplacementFromage(bool direction, int16_t vitesse, int16_t acceleration);

void setup()
{
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
  pinMode(C_FC_deplacementFromage_fin, INPUT_PULLUP);
  // pinMode(BP_man, INPUT_PULLUP);
  // pinMode(BP_auto, INPUT_PULLUP);
  // pinMode(BP_tranche, INPUT_PULLUP);
  // pinMode(BP_cube, INPUT_PULLUP);
  // pinMode(BP_batonnet, INPUT_PULLUP);
  pinMode(BP_start, INPUT_PULLUP);
  pinMode(BP_stop, INPUT_PULLUP);
  pinMode(BP_resetPosition, INPUT_PULLUP);
  // pinMode(BP_manu_avancer_M_avancer, INPUT_PULLUP);
  // pinMode(BP_manu_reculer_M_avancer, INPUT_PULLUP);
  pinMode(C_porteOuverte, INPUT_PULLUP);
  pinMode(C_courant, INPUT_PULLUP);

  // on set les variable comme étant des OutPuts
  // pinMode(M_grille, OUTPUT);
  // pinMode(M_avancer, OUTPUT);
  // pinMode(V_gabari_entre1, OUTPUT);
  // pinMode(V_gabari_entre2, OUTPUT);
  // pinMode(V_gabari_PWM, OUTPUT);
  //   pinMode(M_decoupe1, OUTPUT);
  //   pinMode(M_decoupe2, OUTPUT);
  pinMode(Led_marche, OUTPUT);
  pinMode(Led_erreur, OUTPUT);
  pinMode(ENABLE, OUTPUT);

  digitalWrite(ENABLE, LOW); // Low to enable

  stepper_M_decoupe1.connectToPins(M_decoupe1, D_M_decoupe1);
  stepper_M_decoupe2.connectToPins(M_decoupe2, D_M_decoupe2);
  stepper_M_Fromage.connectToPins(M_avancer, D_M_avancer);
  stepper_M_grille.connectToPins(M_grille, D_M_grille);

  attachInterrupt(digitalPinToInterrupt(BP_AU), arretUrgence, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BP_stop), stop, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C_porteOuverte), PorteOuverte, FALLING);
  flagArretUrgence = LOW; // aru désactivé
  flagStop = LOW;         // stop désactivé
  flagPorteOuverte = LOW; // porte fermée, cad capteur actif

  S0->addTransition(&transitionS0S1, S1);
  S1->addTransition(&transitionS1S2, S2);
  S1->addTransition(&transitionS1S3, S3);

  S2->addTransition(&transitionS2S3, S3);

  S3->addTransition(&transitionS3S4, S4);
  S3->addTransition(&transitionS3S5, S5);
  S3->addTransition(&transitionS3S6, S6);

  S4->addTransition(&transitionS4S7, S7);
  S5->addTransition(&transitionS5S7, S7);
  S6->addTransition(&transitionS6S7, S7);

  S7->addTransition(&transitionS7S8, S8);
  S8->addTransition(&transitionS8S9, S9);
  S9->addTransition(&transitionS9S10, S10);
  S10->addTransition(&transitionS10S11, S11);

  S11->addTransition(&transitionS11S3, S3);

  // S5->addTransition(&transitionS6S0,S0);
}

//=======================================
void state0()
{
  if (machine.executeOnce)
  {
    Serial.println(F("State 0, Attente du Reset"));
    // remonté du vérin
  }
}
bool transitionS0S1()
{
  // if (digitalRead(BP_start)==0)
  // {
  //   Serial.println("state0 =>state1");/* code */
  // }

  if (BP_Reset_Obj.released())
  {
    Serial.println(F("state0 =>state1"));
    return true;
  }
}

void state1_Homing()
{
  // Serial.println("State 0");
  // remonté du vérin
  if (machine.executeOnce)
  {
    Serial.println(F("state1_Homing"));
    digitalWrite(Led_erreur, LOW);
    digitalWrite(Led_marche, HIGH);

    // force l utilisateur a valider son choix au DEBUT du programme UNIQUEMENT (ne sera plus lu après)
    // Int_P_tranche = digitalRead(BP_tranche);
    // E_BP_batonnet = digitalRead(BP_batonnet);
    // E_BP_cube = digitalRead(BP_cube);
    // remontée du vérin
    GabariMotor.setSpeed(100);
    GabariMotor.backward();

    stepper_M_decoupe1.setSpeedInStepsPerSecond(1000);
    stepper_M_decoupe1.setAccelerationInStepsPerSecondPerSecond(1000);
    stepper_M_decoupe1.setTargetPositionInSteps(-20000);

    stepper_M_decoupe2.setSpeedInStepsPerSecond(1000);
    stepper_M_decoupe2.setAccelerationInStepsPerSecondPerSecond(1000);
    stepper_M_decoupe2.setTargetPositionInSteps(-20000);

    stepper_M_grille.setSpeedInStepsPerSecond(600);
    stepper_M_grille.setAccelerationInStepsPerSecondPerSecond(500);
    stepper_M_grille.setTargetPositionInSteps(20000);

    stepper_M_Fromage.setSpeedInStepsPerSecond(600);
    stepper_M_Fromage.setAccelerationInStepsPerSecondPerSecond(500);
    stepper_M_Fromage.setTargetPositionInSteps(20000);
  }

  if (C_FC_verin_entre_Obj.read() == false)
  {
    GabariMotor.stop();
    E_C_FC_verin_entre = true;
  }
  if (C_FC_descenteFil_haut1_Obj.read() == false)
  {
    stepper_M_decoupe1.setAccelerationInStepsPerSecondPerSecond(100000);
    stepper_M_decoupe1.setTargetPositionToStop();
  }

  if (C_FC_descenteFil_haut2_Obj.read() == false)
  {
    stepper_M_decoupe2.setAccelerationInStepsPerSecondPerSecond(100000);
    stepper_M_decoupe2.setTargetPositionToStop();
  }
  if (C_pos_grille1_Obj.read() == false)
  {
    stepper_M_grille.setAccelerationInStepsPerSecondPerSecond(100000);
    stepper_M_grille.setTargetPositionToStop();
  }

  if (C_FC_deplacementFromage_debut_Obj.read() == false)
  {
    stepper_M_Fromage.setAccelerationInStepsPerSecondPerSecond(100000);
    stepper_M_Fromage.setTargetPositionToStop();
  }

  if (!stepper_M_decoupe1.motionComplete())
  {
    stepper_M_decoupe1.processMovement();
    // Serial.print("303: ");
  }
  else
    E_C_FC_descenteFil_haut1 = true;

  if (!stepper_M_decoupe2.motionComplete())
  {
    stepper_M_decoupe2.processMovement();
    // Serial.print("303: ");
  }
  else
    E_C_FC_descenteFil_haut2 = true;

  if (!stepper_M_grille.motionComplete())
  {
    stepper_M_grille.processMovement();
    // Serial.print("303: ");
  }
  else
    E_C_pos_grille1 = true;

  if (!stepper_M_Fromage.motionComplete())
  {
    stepper_M_Fromage.processMovement();
    // Serial.print("303: ");
  }
  else
    E_C_FC_deplacementFromage_debut = true;

  // if (digitalRead(C_FC_descenteFil_haut1) == HIGH)
  // {

  //   //    while(!processMovement())
  //   // {
  //   //   if (digitalRead(homeLimitSwitchPin) == LOW)
  //   //   {
  //   //     limitSwitchFlag = true;
  //   //     directionOfMotion = 0;
  //   //     break;
  //   //   }
  //   // }

  //   // Serial.print(digitalRead(C_FC_descenteFil_haut1));
  //   // stepper_M_decoupe1.setTargetPositionToStop();
  //   stepper_M_decoupe1.setTargetPositionInSteps(stepper_M_decoupe1.getCurrentPositionInSteps());
  //   stepper_M_decoupe1.processMovement();
  //   stepper_M_decoupe1.setCurrentPositionInSteps(0);
  //   E_C_FC_descenteFil_haut1 = true;
  // }
  // else
  // {
  //   stepper_M_decoupe1.processMovement();
  //   // Serial.print("processMovement()");
  // }

  // if (digitalRead(C_FC_descenteFil_haut2) == HIGH)
  // {

  //   stepper_M_decoupe2.setCurrentPositionInSteps(0);
  //   // stepper_M_decoupe2.setTargetPositionToStop();
  //   E_C_FC_descenteFil_haut2 = true;
  // }
  // else
  // {
  //   stepper_M_decoupe2.processMovement();
  // }

  // if (digitalRead(C_pos_grille1) == HIGH)
  // {
  //   stepper_M_grille.setCurrentPositionInSteps(0);
  //   // stepper_M_grille.setTargetPositionToStop();
  //   E_C_pos_grille1 = true;
  // }
  // else
  // {
  //   stepper_M_grille.processMovement();
  // }

  // // positionInitiale();
  // // Serial.println("position initiale atteinte");
  // // Etape = 1;
  // // BP_start
}
bool transitionS1S2() // S2 ==> mode manu
{

  //  if (button1.released()){
  //   Serial.println("state0 =>state1");
  if (E_C_FC_descenteFil_haut1 &&
      E_C_FC_descenteFil_haut2 &&
      E_C_pos_grille1 &&
      E_C_FC_deplacementFromage_debut &&
      E_C_FC_verin_entre &&
      Int_Manu_Obj.read() == false)
  {
    Serial.println(F("state1 =>state2"));
    return true;
  }

  // return E_C_FC_descenteFil_haut1 && E_C_FC_descenteFil_haut2 && E_C_pos_grille1;

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

void state2_ModeManu()
{
  if (machine.executeOnce)
  {
    Serial.println(F("state2_ModeManu"));
  }
  // stepper_M_avancer.processMovement();
}

bool transitionS1S3() // S3 ==> mode auto
{

  if (E_C_FC_descenteFil_haut1 &&
      E_C_FC_descenteFil_haut2 &&
      E_C_pos_grille1 &&
      E_C_FC_deplacementFromage_debut &&
      E_C_FC_verin_entre &&
      Int_Auto_Obj.read() == false)
  {
    Serial.println(F("state1 =>state3"));
    return true;
  }
}

bool transitionS2S3() // S3 ==> mode auto
{

  if (Int_Auto_Obj.read() == false)
  {
    Serial.println(F("state2 =>state3"));
    return true;
  }
}

void state3_ModeAuto()
{
  if (machine.executeOnce)
  {
    Serial.println(F("state3_ModeAuto"));
  }
}

bool transitionS3S4()
{

  if (Int_Tranches_Obj.read() == false)
  {
    return true; /* code */
  }

  //  if (button1.released()){
  //   Serial.println("state0 =>state1");
  // return E_C_FC_descenteFil_haut1||E_C_FC_descenteFil_haut2||E_C_pos_grille1;
  // }
}

void state4_PositionTranches()
{
  if (machine.executeOnce)
  {
    Serial.println(F("state4_PositionTranches"));
    stepper_M_grille.setSpeedInStepsPerSecond(10);
    stepper_M_grille.setAccelerationInStepsPerSecondPerSecond(500);
    stepper_M_grille.setTargetPositionInSteps(200);
  }
  stepper_M_grille.processMovement();

  // A tester de ne pas utiliser les fin de courses

  // if (C_pos_grille1_Obj.read() == false)
  // {
  //   stepper_M_grille.setAccelerationInStepsPerSecondPerSecond(100000);
  //   stepper_M_grille.setTargetPositionToStop();
  // }
}

bool transitionS3S5()
{

  if (Int_Cubes_Obj.read() == false)
  {
    return true; /* code */
  }
}

void state5_PositionCubes()
{
  if (machine.executeOnce)
  {
    Serial.println(F("state5_PositionCubes"));
    stepper_M_grille.setSpeedInStepsPerSecond(10);
    stepper_M_grille.setAccelerationInStepsPerSecondPerSecond(500);
    stepper_M_grille.setTargetPositionInSteps(400);
  }
  stepper_M_grille.processMovement();
}

bool transitionS3S6()
{
  if (Int_Batonnets_Obj.read() == false)
  {
    return true; /* code */
  }
}

void state6_PositionBatonnet() // remontée gabari et fil
{
  if (machine.executeOnce)
  {
    Serial.println(F("state6_PositionBatonnet"));
    stepper_M_grille.setSpeedInStepsPerSecond(10);
    stepper_M_grille.setAccelerationInStepsPerSecondPerSecond(500);
    stepper_M_grille.setTargetPositionInSteps(600);
  }
  stepper_M_grille.processMovement();
}

bool transitionS4S7()
{

  return stepper_M_grille.motionComplete();
}

bool transitionS5S7()
{

  return stepper_M_grille.motionComplete();
}

bool transitionS6S7()
{

  return stepper_M_grille.motionComplete();
}

void state7_Attente() // remontée gabari et fil
{
  if (machine.executeOnce)
  {
    Serial.println(F("state7_Attente"));
  }
}

bool transitionS7S8()
{

  return BP_Start_Obj.released();
}

void state8_AvancementFromage() // remontée gabari et fil
{
  if (machine.executeOnce)
  {
    Serial.println(F("state8_AvancementFromage"));
    stepper_M_Fromage.setSpeedInStepsPerSecond(10);
    stepper_M_Fromage.setAccelerationInStepsPerSecondPerSecond(500);
    stepper_M_Fromage.setTargetPositionInSteps(600);
  }
  stepper_M_Fromage.processMovement();
}

bool transitionS8S9()
{

  return stepper_M_Fromage.motionComplete();
}

void state9_DescendreTranchause() // remontée gabari et fil
{
  if (machine.executeOnce)
  {
    Serial.println(F("state7_Attente"));
    stepper_M_decoupe1.setSpeedInStepsPerSecond(10);
    stepper_M_decoupe1.setAccelerationInStepsPerSecondPerSecond(500);
    stepper_M_decoupe1.setTargetPositionInSteps(600);

    stepper_M_decoupe2.setSpeedInStepsPerSecond(10);
    stepper_M_decoupe2.setAccelerationInStepsPerSecondPerSecond(500);
    stepper_M_decoupe2.setTargetPositionInSteps(600);
  }
  stepper_M_decoupe1.processMovement();
  stepper_M_decoupe2.processMovement();
}

bool transitionS9S10()
{

  return (stepper_M_decoupe1.motionComplete() && stepper_M_decoupe2.motionComplete());
}

void state10_RemontonterTranchauseDescendreGabary() // remontée gabari et fil
{
  if (machine.executeOnce)
  {
    Serial.println(F("state10_RemontonterTranchauseDescendreGabary"));

    stepper_M_decoupe1.setTargetPositionInSteps(0);
    stepper_M_decoupe2.setTargetPositionInSteps(0);
    GabariMotor.setSpeed(255);
    GabariMotor.forward();
  }
  stepper_M_decoupe1.processMovement();
  stepper_M_decoupe2.processMovement();
}

bool transitionS10S11()
{
  static char step = 0;

  switch (step)
  {
  case 0:
  {
    if (C_FC_verin_sorti_Obj.read()==false)
    {
      GabariMotor.stop();
      mytimer.set(200);
      step = 1;
      break;
    }
  }

  case 1:
    if (mytimer.done())
    {
      step = 0;
      return true;
    }
    break;
  }
}

void state11_RemontonterGabary() // remontée gabari et fil
{
  if (machine.executeOnce)
  {
    Serial.println(F("state11_RemontonterGabary"));
    GabariMotor.setSpeed(255);
    GabariMotor.backward();
  }
}

bool transitionS11S3()
{
  if (C_FC_verin_sorti_Obj.read() == false && stepper_M_decoupe1.motionComplete() && stepper_M_decoupe2.motionComplete())
    return true;
}

void loop()
{
  machine.run();

  // // partie calibration
  // E_BP_manu_avancer_M_avancer = digitalRead(BP_manu_avancer_M_avancer);
  // E_BP_manu_reculer_M_avancer = digitalRead(BP_manu_reculer_M_avancer);

  // // mode manu
  // while (digitalRead(BP_man) == 0 && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW) // read bp man est il utile?
  // {
  //   while (digitalRead(BP_manu_reculer_M_avancer) == LOW && digitalRead(C_FC_deplacementFromage_debut) == 0)
  //   {
  //     {
  //       digitalWrite(D_M_avancer, HIGH);
  //       digitalWrite(M_avancer, HIGH);
  //       delay(1); // microstepping activé
  //       digitalWrite(M_avancer, LOW);
  //       delay(1);
  //     }
  //   }
  //   while (digitalRead(BP_manu_avancer_M_avancer) == LOW && digitalRead(C_FC_deplacementFromage_fin) == 0)
  //   {
  //     {
  //       digitalWrite(D_M_avancer, LOW);
  //       digitalWrite(M_avancer, HIGH);
  //       delay(1);
  //       digitalWrite(M_avancer, LOW);
  //       delay(1);
  //     }
  //   }
  // }
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

  // remontée du vérin
  while (digitalRead(C_FC_verin_entre) == 0 && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW)
  {
    digitalWrite(V_gabari_entre1, LOW);
    digitalWrite(V_gabari_entre2, HIGH);
    analogWrite(V_gabari_PWM, 255);
  }
  Serial.println("Vérin monté");

  // remontée du fil
  E_C_FC_descenteFil_haut1 = digitalRead(C_FC_descenteFil_haut1);
  E_C_FC_descenteFil_haut2 = digitalRead(C_FC_descenteFil_haut2);

  while ((digitalRead(C_FC_descenteFil_haut1) == 0 || digitalRead(C_FC_descenteFil_haut2) == 0) && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW)
  {
    Serial.println("trancheuse activée");

    while (digitalRead(C_FC_descenteFil_haut1) == 0 && digitalRead(C_FC_descenteFil_haut2) == 1 && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW)
    {

      digitalWrite(D_M_decoupe1, HIGH); // monter
      digitalWrite(M_decoupe1, HIGH);
      digitalWrite(M_decoupe2, HIGH);
      delayMicroseconds(571);
      digitalWrite(M_decoupe1, LOW);
      delayMicroseconds(571);
    }
    while (digitalRead(C_FC_descenteFil_haut1) == 1 && digitalRead(C_FC_descenteFil_haut2) == 0 && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW)
    {
      digitalWrite(D_M_decoupe2, HIGH);
      digitalWrite(M_decoupe1, HIGH);
      digitalWrite(M_decoupe2, HIGH);
      delayMicroseconds(571);
      digitalWrite(M_decoupe2, LOW);
      delayMicroseconds(571);
    }
    while (digitalRead(C_FC_descenteFil_haut1) == 0 && digitalRead(C_FC_descenteFil_haut2) == 0 && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW)
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
  // retour a la position initiale de la grille
  while (digitalRead(C_pos_grille1) == 0 && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW)
  {

    digitalWrite(D_M_grille, LOW); // recule
    digitalWrite(M_grille, HIGH);
    delay(1);
    digitalWrite(M_grille, LOW);
    delay(1);
  }
  Serial.println("Grille rentrée");
}

// void trancheuse()
// {
//   // descente de la trancheuse
//   if ((((digitalRead(C_pos_grille3) && E_BP_batonnet == 0) || (E_BP_cube == 0 && digitalRead(C_pos_grille2)) || (Int_P_tranche == 0 && digitalRead(C_pos_grille1)))) && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW)
//   {
//     Serial.print("Num etape: ");
//     Serial.println(Etape);
//     // 8700 * 34 800
//     while (NbPas_M_decoupe1 < 34800 && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW) // 200*8cm les 200 en plus sont pour desendre le fil au plus bas
//     {
//       digitalWrite(D_M_decoupe1, LOW); // descend
//       digitalWrite(D_M_decoupe2, LOW);
//       digitalWrite(M_decoupe1, HIGH);
//       digitalWrite(M_decoupe2, HIGH);
//       delayMicroseconds(571);
//       digitalWrite(M_decoupe1, LOW);
//       digitalWrite(M_decoupe2, LOW);
//       delayMicroseconds(571);
//       NbPas_M_decoupe1++;
//     }
//   }

//   // remontée de la trancheuse
//   if (NbPas_M_decoupe1 >= 34800 && flagArretUrgence == LOW && flagStop == LOW && flagPorteOuverte == LOW)
//   {
//     Serial.print("Num etape: ");
//     Serial.println(Etape);
//     while (digitalRead(C_FC_descenteFil_haut1) == 0)
//     {
//       digitalWrite(D_M_decoupe1, HIGH); // monte
//       digitalWrite(D_M_decoupe2, HIGH);
//       digitalWrite(M_decoupe1, HIGH);
//       digitalWrite(M_decoupe2, HIGH);
//       delayMicroseconds(571);
//       digitalWrite(M_decoupe1, LOW);
//       digitalWrite(M_decoupe2, LOW);
//       delayMicroseconds(571);
//     }
//   }
// }

// MOTEUR 1 ET 2 SACAD2 L UN APRES L AUTRE
//  if(BP_start == 1)
//  {
//    long nbStep = 200;
//    stepper_M_decoupe1.setSpeedInStepsPerSecond(200);// nb step/sec
//    stepper_M_decoupe1.setAccelerationInStepsPerSecondPerSecond(1600);
//    stepper_M_decoupe1.moveRelativeInSteps(nbStep); //200 step par 2 mm
//    stepper_M_decoupe2.setSpeedInStepsPerSecond(200);
//    stepper_M_decoupe2.setAccelerationInStepsPerSecondPerSecond(1600);
//    stepper_M_decoupe2.moveRelativeInSteps(nbStep);
//  }

// AVANCE JUSQU AU FIN DE COURSE
//  stepper_M_decoupe1.moveToHomeInRevolutions(Valeur_D_M_decoupe1, 10, 1000, BP_start);
//  stepper_M_decoupe2.moveToHomeInRevolutions(Valeur_D_M_decoupe1, 10, 1000, BP_resetPosition);

// SI FC PAS ACTIVE, MOTEUR 1 ET 2 FONCTIONNE EN MEME TEMPS
//    while(E_BP_stop == 1 && E_BP_resetPosition == 1)
//    {
//    digitalWrite(D_M_decoupe1, HIGH);
//    digitalWrite(D_M_decoupe2, HIGH);
//    digitalWrite(M_decoupe1, HIGH);
//    digitalWrite(M_decoupe2, HIGH);
//    delay(1);
//    digitalWrite(M_decoupe1, LOW);
//    digitalWrite(M_decoupe2, LOW);
//    delay(1);
//    E_BP_stop = digitalRead(BP_stop);
//    E_BP_resetPosition = digitalRead(BP_resetPosition);
//    }

// MOTEUR AVANCE FAST
//   E_BP_start = digitalRead(BP_start);
//    E_BP_resetPosition = digitalRead(BP_resetPosition);
//    if(E_BP_start == 0)
//    {
//      digitalWrite(Led_marche,HIGH);
//     long nbStep = -20000; //- reculer +avancé
//    stepper_M_avancer.setSpeedInStepsPerSecond(400);// nb step/sec
//    stepper_M_avancer.setAccelerationInStepsPerSecondPerSecond(1600);
//    stepper_M_avancer.moveRelativeInSteps(nbStep); //200 step par 2 mm
//    }

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

// MOTEUR AVANCE FAST
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

// if(Etape == 3 && ((digitalRead(E_C_pos_grille3) == 0 && E_BP_batonnet == 0)||(digitalRead(E_C_pos_grille2) == 0 && E_BP_cube == 0)||(digitalRead(E_C_pos_grille1) == 0 && Int_P_tranche == 0)) )
// {
//   while()

// }
