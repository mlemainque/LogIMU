// Filtre de Kalman appliqué à un quaternion, sa dérivée, et le zéro des gyromètres
// Matthias Lemainque 2013

#include "kalman.h"


//  * * * * * * * * * * * * *
// I N I T I A L I S A T I O N
//  * * * * * * * * * * * * *

KALMAN::KALMAN(SENSORS *newSensors) {
  Sensors = newSensors;
}

void KALMAN::setup() {
  // Au début on accepte un écart-type 1000x fois supérieur au paramètres,
  // puis le faisons décroitre selon une exponentielle décroissante, ce pour
  // que le vecteur d'état s'initialise sans utiliser de procédure spécifique
  ETfact = 1000;

  FillA(P, 11*11, 0);
  FillA(X, 11, 0);
  X[0] = 1;
  
  Q = { VQ, VQ, VQ, VQ, VQ, VQ, VQ, VQ, VB, VB, VB };
  R = { Va, Va, Va, Vg, Vg, Vg, Vm, Vm, Vm };  
}


//  * * * * * * * * * * * * * * * * * * * * * *
// G E N E R A T I O N   D E S   M A T R I C E S
//  * * * * * * * * * * * * * * * * * * * * * *

// Voir la feuille de calcul Maple pour les matrices

// Génération de la matrice de prédiction A
// X(t+dt) = A(Y,dt).X(t)
void KALMAN::genA() {
  FillA(AH, 11*11, 0);

  float f, g;
  // Il faut que le quaternion reste normé ; on divise donc par la norme du quaternion et par la norme du quaternion (1+dt*Omega/2)
  g = 1 / sqrt( ( sq(X[0])+sq(X[1])+sq(X[2])+sq(X[3]) )*( 1+sq((*Sensors).dt/2*Y[3])+sq((*Sensors).dt/2*Y[4])+sq((*Sensors).dt/2*Y[5]) ) );
  f = g * (*Sensors).dt/2;

  // Identité partielle
  // On s'arrange pour conserver la norme unitaire de Q=X[0..3]
  for (uint8 i=0 ; i<4  ; i++) AH[12*i] = g;
  for (uint8 i=8 ; i<11 ; i++) AH[12*i] = 1;

  // 1ère ligne (autre que l'identité)
  AH[1] = - f * Y[3];
  AH[2] = - f * Y[4];
  AH[3] = - f * Y[5];

  // 2e ligne (on complètera par antisymétrie)
  AH[13] =   f * Y[5];
  AH[14] = - f * Y[4];

  // 3e ligne
  AH[25] =   f * Y[3];

  // On complète par antisymétrie et on remplit au passage le 2e carré
  for (uint8 i=0 ; i<4 ; i++) { // ligne
    for (uint8 j=i+1 ; j<4 ; j++) { // colonne
      AH[11*(i+4)+j] =   AH[11*i+j] / (*Sensors).dt;
      AH[11*j+i]     = - AH[11*i+j];
      AH[11*(j+4)+i] = - AH[11*(i+4)+j];
    }
  }
}


// Génération de la matrice d'observation H
// Calcule également les projections des mesures dans le référentiel terrestre
// Y(t) = H(X).X(t)
void KALMAN::genH() {
  FillA(AH, 11*11, 0);
  FillA(measureADXL345_0, 3, 0);
  FillA(measureMAG3110_0, 3, 0);

  // Matrice T tq uZ(centrale/0) = TQ
  T = { 
    -X[2],  X[3], -X[0],  X[1],
     X[1],  X[0],  X[3],  X[2],
     X[0], -X[1], -X[2],  X[3] };
  AddMLoc(AH, 9, 11, 1, T, 3, 4, 9.81, 0, 0); // g
  AddMLoc(AH, 9, 11, 1, T, 3, 4, 43.23, 6, 0); // mN
  
  // On calcule la projection des mesures
  PrdM(T2, T, false, X, false, 4, 3, 1); // T2=uZ(centrale/0)
  AddM(measureADXL345_0, 1, T2, 9.81, 3, 1);
  AddM(measureMAG3110_0, 1, T2, 43.23, 3, 1);

  // Matrice T tq uX(centrale/0) = TQ
  T = {  
     X[0],  X[1], -X[2], -X[3],
    -X[3],  X[2],  X[1], -X[0],
     X[2],  X[3],  X[0],  X[1] };
  AddMLoc(AH, 9, 11, 1, T, 3, 4, -20.74, 6, 0); // mT

  PrdM(T2, T, false, X, false, 4, 3, 1); // T2=uX(centrale/terre)
  AddM(measureMAG3110_0, 1, T2, -20.74, 3, 1);

  // Matrice tq Omega(centrale) = 2TdQ
  // Le produit Q_dQ est quaternion pur
  T = { 
    -X[1],  X[0],  X[3], -X[2],
    -X[2], -X[3],  X[0],  X[1],
    -X[3],  X[2], -X[1],  X[0] };
  AddMLoc(AH, 9, 11, 1, T, 3, 4, 2, 3, 4); // On ajoute 2T

  // On rajoute le biais
  T = { 1, 1, 1 };
  AddMDiagLoc(AH, 9, 11, T, 1, 3, 3, 8);
}


//  * * * * * * * * * * * * * * * * * *
// E X E C U T I O N   D U   F I L T R E
//  * * * * * * * * * * * * * * * * * *

void KALMAN::loop() {
  
  lowPassTmp( &ETfact, 1, 2, (*Sensors).dt );
  
  for (uint8 i=0 ; i<3 ; i++) {
    Y[i]   = (*Sensors).measureADXL345[i];
    Y[i+3] = (*Sensors).measureITG3200[i];
    Y[i+6] = (*Sensors).measureMAG3110[i];
  }

  // *****************************
  // Phase de prédiction, ici AH=A

  this->genA();

  PrdM(T, AH, false, X, false, 11, 11, 1); // T=AX
  CopyA(X, T, 11); // X=AX
  
  PrdM(T, AH, false, P, false, 11, 11, 11); // T=AP
  PrdM(P, T, false, AH, true, 11, 11, 11); // P=TAt=APAt
  AddMDiagLoc(P, 11, 11, Q, ETfact, 11, 0, 0); // P=APAt+Q' avec Q'=Q*ETfact

  // *****************************

  // Ici, Q est parfaitement normé
  
  CalcCardan(Cardan, X); // On calcule les angles de Cardan

  // ******************************
  // Phase de mise à jour, ici AH=H

  this->genH();

  PrdM(K, AH, false, P, false, 9, 11, 11); // K=HP
  PrdM(T, K, false, AH, true, 9, 11, 9);  // T=KHt=HPHt
  AddMDiagLoc(T, 9, 9, R, 1, 9, 0, 0); // T=HPHt+R
  InvM(T, 9); // T=(HPHt+R)^(-1)
  PrdM(K, AH, true, T, false, 11, 9, 9); // K=HtT=Ht(HPHt+R)^(-1)
  PrdM(T, P, false, K, false, 11, 11, 9); // K=PHt(R'+HPHt)^(-1)
  CopyA(K, T, 11*9);

  PrdM(T2, AH, false, X, false, 9, 11, 1); // T=HX
  AddM(T2, -1, Y, 1, 9, 1); // T=Y-HX
  PrdM(T, K, false, T2, false, 11, 9, 1); // T=K(Y-HX)
  AddM(X, T, 11, 1); // X=X+K(Y-HX)

  PrdM(T, K, false, AH, false, 11, 9, 11); // T=KH
  for (uint8 i=0 ; i<11 ; i++) { // T=I-KH
    for (uint8 j=0 ; j<11 ; j++) T[11*i+j] = 1*(i==j) - T[11*i+j];
  }
  PrdM(K, T, false, P, false, 11, 11, 11); // K=TP=(I-KH)P
  CopyA(P, K, 11*11); // P=K=(I-KH)P
  // K ne servira plus à rien, on l'utilise ici comme intermédiaire de calcul
  // Ca évite d'avoir à déclarer une 2e matrice temporaire de dimension 11x11 ...

  // ******************************
  
}

