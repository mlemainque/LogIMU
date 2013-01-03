// Matthias Lemainque 2012
// Composant IMU, paramétrable, permettant de calculer une orientation à partir des mesures

#ifndef _IMU_H_
#define _IMU_H_

#include "itg3200.h"
#include "adxl345.h"
#include "mag3110.h"

#include "maths.h"
#include "state.h"


// Vecteurs universels (utilisés par la calibration et par la correction) (sens du vecteur : source -> centrale)
#define Gravitee_0	{0, 0, 9.81}		// Gravitée (m.s-2)
#define NordMagn_0	{-20.74, 0, 43.23}	// Direction du nord magnétique à Paris (uT)
						// Source : http://wdc.kugi.kyoto-u.ac.jp/igrf/point/index.html
Vector Gravitee = Gravitee_0;
Vector NordMagn = NordMagn_0;

// Correction orientation par les accéléromètres
float CorrectA_Const = 1.0;	// Constante de temps de correction dynamique par les accéléromètres (sec)
float CorrectA_ENorm = 0.5;	// Constante de "confiance" en fonction de l'erreur de normes de l'accélération (ratio de normes)
float CorrectA_EDir = 5;	// Constante de "confiance" en fonction de l'erreur de directions de l'accélération (degrés)
float CorrectA_Diff = 0.5;	// Constante de "confiance" lors de fortes fluctuation d'accélération (ratio de normes jerk/acc)

// Correction orientation par les magnétomètres
float CorrectM_Const = 1.5;	// Constante de temps de correction dynamique par les magnétomètres (sec)
float CorrectM_ENorm = 0.5;	// Constante de "confiance" en fonction de l'erreur de normes du champ magnétique (ratio de normes)
float CorrectM_EDir = 5;	// Constante de "confiance" en fonction de l'erreur de directions du champ magnétique (degrés)
float CorrectM_Diff = 0.5;	// Constante de "confiance" lors de fortes fluctuation du champ (ratio de normes)

// Rem : Constante de confiance = valeur pour laquelle la confiance n'est déjà plus que de 75%

class IMU {
public:
  IMU(boolean corA=true, boolean corM=true, boolean en=true, boolean enP=false);
  void lunch();

  // Paramètres
  boolean enabled;
  boolean correctA, correctM;
  boolean posit;

  // Informations à afficher
  float confM, errNormM, errDirM;
  float confA, errNormA, errDirA;

  // Calcul de l'orientation
  Base base;
  void procIMU();
  Vector dataAcc0;
  Vector dataMag0;
  
  // Intégration position
  Vector dataVit0, dataPos0;
};



// Initialisation configuration
IMU::IMU(boolean corA, boolean corM, boolean en, boolean enP) {
  this->enabled = en;
  this->correctA = corA;
  this->correctM = corM;
  this->posit = enP;
}

// Lancement de l'IMU
void IMU::lunch() {
  // Initialisation des variables
  this->base.init();
  this->dataVit0 = newVector(0,0,0);
  this->dataPos0 = newVector(0,0,0);
  this->confA = 0;
  this->confM = 0;
  if (!this->enabled) return;

  // On se recale immédiatement par rapport aux mesures
  //void correction(    Vector n,         Vector source,                          Vector cible, float dt, float tmpConst, Vector dSource, float NdSourceCst, float *errNorm,  float errNormConfCst, float *errDir,  float errDirConfCst, float *factAff, boolean force);
  this->base.correction(newVector(0,0,1), changeRef(mag.measureSlow, this->base), NordMagn,     0,        0,              mag.dMeasure,   999,               &this->errNormM, 999,                  &this->errDirM, 999,                 &this->confM,   true); // Correction forcée en lacet uniquement, par les magnétomètres
  this->base.correction(newVector(0,0,0), changeRef(acc.measureSlow, this->base), Gravitee,     0,        0,              acc.dMeasure,   999,               &this->errNormA, 999,                  &this->errDirA, 999,                 &this->confA,   true); // Correction forcée en 3D par les accéléromètres
}


// Calcule la nouvelle orientation de la base en fonction des mesures
// Exploite l'accélération (respectivement le champ magnétique) pour donner la verticale (le nord magnétique)
// et ainsi éviter un biais dû au mauvais calibrage des gyromètres et à la trop faible fréquence de calcul
void IMU::procIMU() {
  if (!this->enabled) return;

  // Tourne la base en fonction des vitesses de rotation mesurées
  this->base.rotateX(gyr.measure.X*dt);
  this->base.rotateY(gyr.measure.Y*dt);
  this->base.rotateZ(gyr.measure.Z*dt);

  // On repasse l'accélération et le champ magnétique dans le référentiel terrestre
  this->dataMag0 = changeRef(mag.measure, this->base);
  this->dataAcc0 = changeRef(acc.measure, this->base);

  // On veut que l'accélération et le champ magnétique mesurés collent à ceux attendus
  // Pour la correction du nord magnétique, on n'autorise que les rotations dans le plan horizontal

  //                        void correction(Vector n,         Vector source,  Vector cible, float dt, float tmpConst, Vector dSource, float NdSourceCst, float *errNorm,  float errNormConfCst, float *errDir,  float errDirConfCst, float *factAff, boolean force=false);
  if (this->correctM) this->base.correction(newVector(0,0,1), this->dataMag0, NordMagn,     dt,       CorrectM_Const, mag.dMeasure,   CorrectM_Diff,     &this->errNormM, CorrectM_ENorm,       &this->errDirM, CorrectM_EDir,       &this->confM,   false);
  if (this->correctA) this->base.correction(newVector(0,0,0), this->dataAcc0, Gravitee,     dt,       CorrectA_Const, acc.dMeasure,   CorrectA_Diff,     &this->errNormA, CorrectA_ENorm,       &this->errDirA, CorrectA_EDir,       &this->confA,   false);

  // Rétablie l'orthogonalité de la base (une erreure apparait lentement sinon)
  this->base.checkOrthoNorm();
  
  // On intègre éventuellement la position
  if (this->posit) {
    this->dataVit0 = Comb2( this->dataVit0, 1, this->dataAcc0, dt );
    this->dataPos0 = Comb2( this->dataPos0, 1, this->dataVit0, dt );
  }
}





#endif // _IMU_H_






/*

 .... NETOGRAPHIE ................................................................................................
 Capteur acheté				http://www.sparkfun.com/products/10121
 Microprocesseur acheté			http://www.sparkfun.com/products/11280
 Procédures de communication i2c	http://pastebin.com/Z6UAdpvM
 Configuration des capteurs		https://github.com/henryeherman/ee180ucla_maple_demo
 Moyenne glissante exponentielle	http://fr.wikipedia.org/wiki/Moyenne_glissante#Moyenne_mobile_exponentielle
 
 */




