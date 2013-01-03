// Boîte à outils mathématiques
// Matthias Lemainque 2012

#ifndef _MATHS_H_
#define _MATHS_H_

typedef struct {
  float X;
  float Y;
  float Z;
} 
Vector;

typedef struct {
  float lacet;
  float roulis;
  float tangage;
}
Orient;

enum Axis {
  axis_X, axis_Y, axis_Z};

// Conversion degrés/radians
#define CDR 57.295779513

// Valeurs trigonométriques
#define cosPI4 0.7071067810
#define sinPI4 0.7071067810

// Valeurs racines
#define SQ2 1.414213562
#define SQ3 1.732050808

// Produit scalaire v1.v2
float PrdS(Vector v1, Vector v2) {
  return v1.X*v2.X + v1.Y*v2.Y + v1.Z*v2.Z;
}

// Produit vectoriel v1^v2
Vector PrdV(Vector v1, Vector v2) {
  Vector result;
  result.X = v1.Y*v2.Z - v1.Z*v2.Y;
  result.Y = v1.Z*v2.X - v1.X*v2.Z;
  result.Z = v1.X*v2.Y - v1.Y*v2.X;
  return result;
}

// Produit de chaque coordonnées
Vector Prd2(Vector v1, Vector v2) {
  Vector result;
  result.X = v1.X*v2.X;
  result.Y = v1.Y*v2.Y;
  result.Z = v1.Z*v2.Z;
  return result;
}

// Norme d'un vecteur
float Norm(Vector v) {
  return sqrt( PrdS(v,v) );
}

float Norm2(Vector v) {
  return PrdS(v,v);
}

// Multiplication par un réel
Vector Mult(Vector v, float fact) {
  Vector result;
  result.X = v.X * fact;
  result.Y = v.Y * fact;
  result.Z = v.Z * fact;
  return result;
}

// Normalise un vecteur
Vector UnitVect(Vector v) {
  return Mult( v, 1/Norm(v) );
}

// Addition de deux vecteurs
Vector Add2(Vector v1, Vector v2) {
  Vector result;
  result.X = v1.X + v2.X;
  result.Y = v1.Y + v2.Y;
  result.Z = v1.Z + v2.Z;
  return result;
}

// Combinaison linéaire de 2 vecteurs
Vector Comb2(Vector v1, float f1, Vector v2, float f2) {
  return Add2( Mult(v1,f1), Mult(v2,f2) );
}

// Combinaison linéaire de 3 vecteurs
Vector Comb3(Vector v1, float f1, Vector v2, float f2, Vector v3, float f3) {
  return Comb2( Comb2(v1, f1, v2, f2), 1, v3, f3);
}

// Eleve au carré les composantes d'un vecteur
Vector sqV(Vector v) {
  Vector result;
  result.X = sq(v.X);
  result.Y = sq(v.Y);
  result.Z = sq(v.Z);
  return result;
}

// Prend la racine carrée des composantes d'un vecteur
Vector sqrtV(Vector v) {
  Vector result;
  result.X = sqrt(v.X);
  result.Y = sqrt(v.Y);
  result.Z = sqrt(v.Z);
  return result;
}

// Rotation d'un seul vecteur, en donnant directement le sinus et le cosinus de l'angle
Vector RotateVect(Vector v, Vector axe, float c, float s) {
  return Comb3(v, c, axe, (1-c)*PrdS(axe,v), PrdV(axe,v), s);
}

// Crée un nouveau vecteur
Vector newVector(float X, float Y, float Z) {
  Vector result;
  result.X = X;
  result.Y = Y;
  result.Z = Z;
  return result;
}

// Teste l'égalité de deux vecteurs
boolean idVector(Vector v1, Vector v2) {
  return ((v1.X==v2.X) && (v1.Y==v2.Y) && (v1.Z==v2.Z));
}

// Coordonnées extreme d'un vecteur
float maxV(Vector v) {
  return max(max(v.X,v.Y),v.Z);
}

// Projection sur le plan de normale n (unitaire !)
Vector ProjVect(Vector v, Vector n) {
  return PrdV( n, PrdV(v,n) );
}

// Convertit une mesure brute 16 bits (ici 10 bits) en réel
Vector convertVector(Vector v, float fact, Vector zero) {
  Vector result=v;
  if (v.X>=32768) result.X=v.X-65536;
  if (v.Y>=32768) result.Y=v.Y-65536;
  if (v.Z>=32768) result.Z=v.Z-65536;
  return Comb2(result, fact, zero, -1);
}


// Filtre passe-bas récursif
// Pour un 1er ordre de constante de temps T, il faut prendre cst=dt/T
void lowPass(float *value, float newValue, float cst) {
  float cst2 = constrain( cst, 0, 1 );
  *value = newValue*cst2 + *value*(1-cst2);
}

void lowPassTmp(float *value, float newValue, float cst, float dt) {
  if (cst==0) *value = newValue;
  else lowPass(value, newValue, dt/cst);
}

void lowPassVect(Vector *value, Vector newValue, float cst) {
  lowPass(&((*value).X), newValue.X, cst);
  lowPass(&((*value).Y), newValue.Y, cst);
  lowPass(&((*value).Z), newValue.Z, cst);
}



class Base {
private:
  void rotate(Vector axe, float c, float s);
public:
  Base();
  void init();
  Vector uX;
  Vector uY;
  Vector uZ;
  void rotateX(float a);
  void rotateY(float a);
  void rotateZ(float a);
  void correction(Vector n, Vector source, Vector cible, float dt, float tmpConst, Vector dSource, float NdSourceCst, float *errNorm, float errNormConfCst, float *errDir, float errDirConfCst, float *factAff, boolean force);
  void checkOrthoNorm();
  Orient orient();
};

Base::Base() {
  this->init();
}

void Base::init() {
  this->uX = newVector(1,0,0);
  this->uY = newVector(0,1,0);
  this->uZ = newVector(0,0,1);
}

void Base::rotate(Vector axe, float c, float s) {
  this->uX = RotateVect( this->uX, axe, c, s );
  this->uY = RotateVect( this->uY, axe, c, s );
  this->uZ = RotateVect( this->uZ, axe, c, s );
}

void Base::rotateX(float a) {
  this->rotate(this->uX, cos(a/CDR), sin(a/CDR));
}

void Base::rotateY(float a) {
  this->rotate(this->uY, cos(a/CDR), sin(a/CDR));
}

void Base::rotateZ(float a) {
  this->rotate(this->uZ, cos(a/CDR), sin(a/CDR));
}


// Fonction ressemblant à la répartition de Gauss mais plus rapide à calculer, dérivable (x>0)
// x2 est censé être une grandeur au carré : cette complication évite de prendre la racine puis de finalement élever au carré dans la fonction
// (deux morceaux : une parabole inverse et une hyperbole)
float confianceFonc(float x2) {
  if (x2<=0) return 1;
  else if (x2<=2) return 1-x2/4;
  else return 1/x2;
}

// Tourne la base de façon à ce que Source vienne se coller sur Cible
// Projète éventuellement perpendiculairement à n (unitaire !) à moins que n soit nul
// Fait varier le degré de correction selon l'erreur de normes et de direction (utilise la fonction confianceFonc() )
void Base::correction(Vector n,       Vector source,  Vector cible,      float dt, float tmpConst,    Vector dSource, float NdSourceCst,	float *errNorm,      float errNormConfCst,    float *errDir,	    float errDirConfCst,     float *factAff,	       boolean force) {
  //		      Vecteur normal  Vecteur mesuré  Vecteur théorique	 dt        Constante de temps Dérivée de      Constante de tolérance	Erreur courante sur  Constante de "confiance" Erreur courante sur   Constante de "confiance" Proportion courante de    Force la correction?
  //		      (projection)    (réf terrestre) (réf terrestre)	 	   de la correction   la source	      sur la norme de dSource	normes (ratio)	     (erreur de norme)	      la direction (degrés) (erreur de direction)    de correction (passe-bas) de correction

  // On calcule les normes/angles et on projète si le plan de correction est imposé
  float nS = Norm(source);
  float nC = Norm(cible);
  if ((nC*nS == 0) && !force) return;				// Si l'un des vecteurs est nul, aucune correction
  if (!idVector(n, newVector(0,0,0))) {				// On a demandé une projection dans un plan
    source = UnitVect( ProjVect( source, n ) );
    cible = UnitVect( ProjVect( cible, n ) );
  }
  else {
    source = Mult( source, 1/nS);
    cible = Mult( cible, 1/nC);
  }
  float c = PrdS(cible, source);			// Cosinus de l'angle de correction complet (cible et source ont été normés)

  // On calcule l'erreur actuelle de normes et de directions
  // et on en déduit le degré de correction à appliquer
  float fact = 1;
  *errNorm = abs( nS - nC ) / nC;
  *errDir = acos(constrain(c, -1, 1)) * CDR; // degrés
  if (!force) {
    fact = min( confianceFonc(sq(*errNorm/errNormConfCst)), confianceFonc(sq(*errDir/errDirConfCst)) ); // Confiance définie par rapport à l'erreur de valeur de source (direction/norme)
    fact = max( confianceFonc(Norm2(dSource)/sq(nC)/sq(NdSourceCst)), fact ); // Mais si dSource est faible alors on considère que l'on peut croire Source quand même
  }

  // On applique la correction
  Vector axe = UnitVect( PrdV(source, cible) );		// Le produit vectoriel normalisé fournit l'axe de rotation
  float fact2 = fact;
  if (!force) fact2 = constrain( fact2 * dt/tmpConst, 0, 1 );	// On prend en compte le constante de temps de correction
  c = constrain( cos( *errDir/CDR*fact2 ), -1, 1);	// On applique le coefficient à l'angle de rotation
  float s = sqrt( 1 - sq(c) );				// On déduit le sinus (l'angle est compris entre 0 et Pi donc le sinus est de toute façon positif)
  this->rotate( axe, c, s );				// On effectue la rotation

  // Passe-bas appliqué au degré de correction (destiné à l'affichage)
  lowPassTmp(factAff, fact, 0.2, dt);
}


// Normalise et orthogonalise (méthode de Schmidt) les vecteurs de la base
void Base::checkOrthoNorm() {
  this->uX = UnitVect( this->uX );
  this->uY = UnitVect( ProjVect( this->uY, this->uX ) );
  this->uZ = PrdV( this->uX, this->uY );
}

// Renvoit l'orientation de la base
// Implique que la base reste relativement "à plat", pour que les angles puissent être interprétés
Orient Base::orient() {
  Orient result;
  result.lacet   = atan2( this->uX.Y , this->uX.X ) *CDR;
  result.roulis  = atan2( this->uX.Z, Norm(ProjVect(this->uX,newVector(0,0,1))) ) *CDR; // angle entre uX et sa projection sur le plan horizontal
  result.tangage = atan2( this->uY.Z, Norm(ProjVect(this->uY,newVector(0,0,1))) ) *CDR; // angle entre uY et sa projection sur le plan horizontal
  return result;
}



// Change de référentiel
// Paramètre : vecteur V dans la base B dans le base C
// Retour : vecteur V' dans la base C
Vector changeRef(Vector v, Base b) {
  return Comb3(b.uX, v.X, b.uY, v.Y, b.uZ, v.Z);
}



int32 lastRand;

void seedRand(uint32 seed) {
  lastRand = seed;
}

float randF() {
  lastRand = (16807*lastRand)%2147483647;
  return lastRand/2147483648;
}



#endif // _MATHS_H_



