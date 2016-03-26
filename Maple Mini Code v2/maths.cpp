// Boîte à outils mathématiques
// Matthias Lemainque 2012

#include "maths.h"

// * * * * * * * * *
//  M A T R I C E S
// * * * * * * * * *

// Remplit un tableau
void FillA(float* A, uint8 n, float f) {
  for (uint8 i=0 ; i<n ; i++) {
    A[i] = f;
  }
}

// Copie un tableau
void CopyA(float* A, float* B, uint8 n) {
  for (uint8 i=0 ; i<n ; i++) {
    A[i] = B[i];
  }
}

// Echange deux tableaux
void ExchangeA(float* A, float* B, uint8 n, float* T) {
  CopyA(T, A, n);
  CopyA(A, B, n);
  CopyA(B, T, n);
}

// Ajoute localement une matrice à une autre (et applique des coeffs)
void AddMLoc(float* A, uint8 mA, uint8 nA, float fA, float* B, uint8 mB, uint8 nB, float fB, uint8 i1, uint8 j1) {
// N'agit que sur la zone de recouvrement de B
  for (uint8 i=0 ; i<mB ; i++) {
    for (uint8 j=0 ; j<nB ; j++) {
      A[(i+i1)*nA+(j+j1)] *= fA;
      A[(i+i1)*nA+(j+j1)] += fB * B[i*nB+j];
    }
  }
}

void AddM(float* A, float fA, float* B, float fB, uint8 m, uint8 n) {
  AddMLoc(A, m, n, fA, B, m, n, fB, 0, 0);
}

void AddM(float* A, float* B, uint8 m, uint8 n) {
  AddMLoc(A, m, n, 1, B, m, n, 1, 0, 0);
}

void AddA(float *A, float fA, float *B, float fB, uint8 n) {
  AddMLoc(A, n, 1, fA, B, n, 1, fB, 0, 0);
}

// Ajoute localement une matrice diagonale à une autre matrice
void AddMDiagLoc(float* A, uint8 mA, uint8 nA, float* D, float fD, uint8 nD, uint8 i1, uint8 j1) {
  for (uint8 i=0 ; i<nD ; i++) {
    A[(i+i1)*nA+(i+j1)] += D[i]*fD;
  }
}

void AddMDiag(float* A, float* D, uint8 n) {
  AddMDiagLoc(A, n, n, D, n, 1, 0, 0);
}


// Produit matriciel C=AB
// A(m,p) B(p,n) C(m,n)
// (matrices de taille 255x255 maxi)
void PrdM(float* C, float* A, boolean At, float* B, boolean Bt, uint8 m, uint8 n, uint8 p) {
  uint8 i, j, k, nA, nB;
  for (i=0 ; i<m ; i++) {
    for (j=0 ; j<p ; j++) {
      C[p*i+j]=0;
      for (k=0 ; k<n ; k++) {
        if (!At) nA=n*i+k; 
        else nA=m*k+i;
        if (!Bt) nB=p*k+j; 
        else nB=n*j+k;
        C[p*i+j] += A[nA] * B[nB];
      }
    }
  }
}

// Combinaison linéaire de matrices C=Aa+Bb
// ABC(m,n)
void Comb2M(float* A, float a, float* B, float b, uint8 m, uint8 n, float* C) {
  uint8 i, j;
  for (i=0 ; i<m ; i++) {
    for (j=0 ; j<n ; j++) {
      C[n*i+j] = A[n*i+j]*a + B[n*i+j]*b;
    }
  }
}

// Inversion matricielle (pivot de Gauss)
// A(n,n) est remplacée par le résultat
// La procédure s'arrête si la matrice s'avère être singulière
void InvM(float* A, int n) {
  int16 k;      
  uint8 i, j, pivrow;
  uint8 pivrows[n];
  float tmp;      

  for (k=0 ; k<n ; k++) {
    // On trouve le plus grand pivot possible pour assurer la stabilité numérique de l'inversion
    tmp = 0;
    for (i=k ; i<n ; i++) {
      if (abs(A[i*n+k]) >= tmp) { 
        tmp = abs(A[i*n+k]);
        pivrow = i;
      }
    }

    if (A[pivrow*n+k] == 0) return; // Matrice singulière car pivot nul

    // Effectue les opérations de pivot sur la matrice initiale
    if (pivrow != k) {
      for (j=0 ; j<n ; j++) {
        tmp = A[k*n+j];
        A[k*n+j] = A[pivrow*n+j];
        A[pivrow*n+j] = tmp;
      }
    }
    pivrows[k] = pivrow;    // record row swap (even if no swap happened)

    // On divise la ligne par le pivot ; la matrice devient en partie celle du résultat
    tmp = 1 / A[k*n+k];
    A[k*n+k] = 1.;      
    for (j=0 ; j<n ; j++) A[k*n+j] = A[k*n+j]*tmp;

    for (i=0 ; i<n ; i++) {
      if (i != k) {
        tmp = A[i*n+k];
        A[i*n+k] = 0;
        for (j=0 ; j<n ; j++) A[i*n+j] = A[i*n+j] - A[k*n+j]*tmp;
      }
    }
  }

  for (k=n-1 ; k>=0 ; k--) { // k doit pouvoir passer négatif pour sortir de la boucle (uint ne convient pas)
    if (pivrows[k] != k) {
      for (i=0 ; i<n ; i++) {
        tmp = A[i*n+k];
        A[i*n+k] = A[i*n+pivrows[k]];
        A[i*n+pivrows[k]] = tmp;
      }
    }
  }
}

float Norm2V(float *vect) {
  return sq(vect[0]) + sq(vect[1]) + sq(vect[2]);
}

float NormV(float *vect) {
  return sqrt( Norm2V(vect) );
}

float PrdSV(float *A, float *B) {
  float PrdS = 0;
  for (uint8 i=0 ; i<3 ; i++) PrdS += A[i] * B[i];
  return PrdS;
}

void PrdVV(float *res, float *A, float *B) {
  for (uint8 i=0 ; i<3 ; i++) res[i] = A[(1+i)%3] * B[(2+i)%3] - A[(2+i)%3] * B[(1+i)%3];
}

void RotV(float *vect, float axeX, float axeY, float axeZ, float c, float s) {
  float axe[3] = {axeX, axeY, axeZ};
  float ProdScal = PrdSV(vect, axe);
  float ProdVect[3];
  PrdVV(ProdVect, axe, vect);
  AddM(vect, c, axe, (1-c)*ProdScal, 3, 1);
  AddM(vect, 1, ProdVect, s, 3, 1);
}

float DLcos(float x) {
  return 1 - sq(x)/2 + sq(sq(x))/6;
}

void CalcCardan(float *res, float* Q) {
  res[0] = atan2( 2* (Q[1]*Q[2]+Q[3]*Q[0]) ,1-2*(sq(Q[2])+sq(Q[3]))) *CDR;	// Lacet
  res[1] = asin(  2* (Q[0]*Q[2]-Q[1]*Q[3]))			     *CDR;	// Roulis
  res[2] = atan2( 2* (Q[0]*Q[1]+Q[2]*Q[3]) ,1-2*(sq(Q[1])+sq(Q[2]))) *CDR;	// Tangage
}



// * * * * * * * * * *
//  P A S S E - B A S
// * * * * * * * * * *

void lowPassTmp(float *value, float newValue, float cstTmp, float dt) {
  if (cstTmp == 0) *value = newValue;
  else {
    float fact = constrain( dt/cstTmp, 0, 1 );
    *value = newValue*fact + *value*(1-fact);
  }
}


// * * * * * * * * * * * * * * * * * *
//  C O N V E R S I O N S   T Y P E S
// * * * * * * * * * * * * * * * * * *

// On veut que les deux valeurs min et max puissent être atteintes, donc le facteur est 1<<n-1 et non 1<<n
uint64 ftoi(const float value, float min, float max, uint8 n) {
  float value2 = constrain(value, min, max);
  return (value2-min)/(max-min) * ((1<<n)-1);
}

float itof(const uint64 value, float min, float max, uint8 n) {
  return (float)(value/((1<<n)-1)) * (max-min) + min;
}

