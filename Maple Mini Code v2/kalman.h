// Filtre de Kalman appliqué à un quaternion, sa dérivée, et le zéro des gyromètres
// Matthias Lemainque 2013

#ifndef _KALMAN_H_
#define _KALMAN_H_

#include "wirish.h"
#include "sensors.h"
#include "maths.h"

//#include "pcd8544.h"

// Paramètres
// Variances maximales
const float VQ = 0.000001;	// Quaternion et sa dérivée
const float VB = 0.1;		// Biais gyroscopique
const float Va = 3;		// Accéléromètres (reglé empiriquement)
const float Vg = 0.004;		// Gyromètres : bruit de 0.38 °/s rms
const float Vm = 7;		// Magnétomètres : bruit de 4 uT rms

class KALMAN {
private:
  SENSORS *Sensors;

  float ETfact;		// Facteur multiplicatif de Q
  float dt;		// Récupéré sur Sensors
  float T[11*11];	// tmp
  float T2[9];		// tmp

  void genA();
  void genH();
  
public:
  KALMAN(SENSORS *newSensors);

  float X[11];		// Vecteur d'état
  float Y[9];		// Vecteur de mesure
  float AH[11*11];	// Matrice de prédiction ( X(k+1)=AX(k) )
                        // Matrice d'observation ( Y=HX )
  float P[11*11];	// Matrice de covariance sur X
  float K[11*11];	// Gain de Kalman

  float Q[11];		// Matrice diag. de covariance maxi sur X (à définir)
  float R[9];		// Matrice diag. de covariance sur Y (à définir)
  
  float Cardan[3];	// Angles de cardan
  
  float measureADXL345_0[3];
  float measureMAG3110_0[3];

  void setup();
  void loop();
  
  boolean gyroOnly;
};

#endif // _KALMAN_H_
