// Boîte à outils mathématiques
// Matthias Lemainque 2012

#ifndef _MATHS_H_
#define _MATHS_H_

#include "wirish.h"

// Conversion degrés/radians
const float CDR = 57.295779513;

// Valeurs trigonométriques
#define COSPI4 0.7071067810
#define SINPI4 0.7071067810

// Opérations sur les matrices
void FillA(float* A, uint8 n, float f); // Remplit un tableau
void Comb2M(float* A, float a, float* B, float b, uint8 m, uint8 n, float* C); // Combinaison linéaire de matrices C=Aa+Bb
void CopyA(float* A, float* B, uint8 n); // Copie un tableau
void ExchangeA(float* A, float* B, uint8 n, float* T); // Echange deux tableaux

void AddMLoc(float* A, uint8 mA, uint8 nA, float fA, float* B, uint8 mB, uint8 nB, float fB, uint8 i1, uint8 j1);
void AddM(float* A, float fA, float* B, float fB, uint8 m, uint8 n);
void AddM(float* A, float* B, uint8 m, uint8 n);
void AddA(float *A, float fA, float *B, float fB, uint8 n);

void AddMDiagLoc(float* A, uint8 mA, uint8 nA, float* D, float fD, uint8 nD, uint8 i1, uint8 j1);
void AddMDiag(float* A, float* D, uint8 n);

void PrdM(float* C, float* A, boolean At, float* B, boolean Bt, uint8 m, uint8 n, uint8 p); // Produit matriciel C=AB
void InvM(float* A, int n); // Inversion matricielle (pivot de Gauss)

float Norm2V(float *vect); // Norme au carré d'un vecteur 3D
float NormV(float *vect); // Norme d'un vecteur 3D
float PrdSV(float *A, float *B); // Produit scalaire de deux vecteurs 3D
void PrdVV(float *res, float *A, float *B); // Produit vectoriel
void RotV(float *vect, float axeX, float axeY, float axeZ, float c, float s); // Rotation vectorielle 3D
void CalcCardan(float *res, float* Q); // Angles de Cardan associés à un quaternion


float DLcos(float x);


// Filtre passe-bas du premier ordre
void lowPassTmp(float *value, float newValue, float cstTmp, float dt);


// Opérations sur les types
uint64 ftoi(const float value, float min, float max, uint8 n=8);
float itof(const uint64 value, float min, float max, uint8 n=8);


#endif // _MATHS_H_
