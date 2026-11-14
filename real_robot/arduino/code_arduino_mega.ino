// ============================================================
// ROBOT MOBILE - ARDUINO MEGA + L298N
//
// Commandes recues du Raspberry Pi :
//   AVANCE
//   GAUCHE
//   DROITE
//   STOP
//
// Messages renvoyes au Raspberry Pi :
//   ACK:AVANCE
//   ACK:GAUCHE
//   ACK:DROITE
//   ACK:STOP
//   BUSY
//   TERMINE
// ============================================================


// ============================================================
// BROCHES L298N
// ============================================================

// Cote droit
const int ENA = 5;
const int IN1 = 22;
const int IN2 = 23;

// Cote gauche
const int ENB = 6;
const int IN3 = 24;
const int IN4 = 25;


// ============================================================
// PARAMETRES MOTEURS
// ============================================================

// Compensation pour corriger la deviation vers la droite
const int vitesseAvanceDroite = 100;
const int vitesseAvanceGauche = 100;

const int vitesseRotation = 100;


// ============================================================
// TEMPS DE ROTATION
// ============================================================

unsigned long tempsRotation25 = 600;


// ============================================================
// ETATS INTERNES
// ============================================================

enum EtatRobot
{
  ARRET_LIBRE,
  AVANCE_CONTINUE,
  ROTATION
};

EtatRobot etat = ARRET_LIBRE;


// ============================================================
// GESTION DU TEMPS
// ============================================================

unsigned long debutEtat = 0;


// ============================================================
// COMMANDE RECUE PENDANT UNE ROTATION
// ============================================================

// Vide = aucune nouvelle commande recue pendant la rotation
char commandeApresRotation[16] = "";


// ============================================================
// BUFFER COMMUNICATION SERIE
// ============================================================

char bufferSerie[32];
byte indexSerie = 0;


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  arreterMoteurs();

  etat = ARRET_LIBRE;
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  lirePortSerie();
  gererMouvement();
}


// ============================================================
// LECTURE SERIE NON BLOQUANTE
// ============================================================

void lirePortSerie()
{
  while (Serial.available() > 0)
  {
    char caractere = Serial.read();

    if (caractere == '\n')
    {
      bufferSerie[indexSerie] = '\0';

      traiterCommande(bufferSerie);

      indexSerie = 0;
    }

    else if (caractere == '\r')
    {
      // Ignorer
    }

    else
    {
      if (indexSerie < sizeof(bufferSerie) - 1)
      {
        bufferSerie[indexSerie] = caractere;
        indexSerie++;
      }

      else
      {
        indexSerie = 0;
      }
    }
  }
}


// ============================================================
// TRAITEMENT DES COMMANDES
// ============================================================

void traiterCommande(const char* commande)
{
  // ==========================================================
  // STOP EST TOUJOURS PRIORITAIRE
  // ==========================================================

  if (strcmp(commande, "STOP") == 0)
  {
    Serial.println("ACK:STOP");

    arreterMoteurs();

    etat = ARRET_LIBRE;

    commandeApresRotation[0] = '\0';

    return;
  }


  // ==========================================================
  // VERIFIER QUE LA COMMANDE EST VALIDE
  // ==========================================================

  if (
      strcmp(commande, "AVANCE") != 0 &&
      strcmp(commande, "GAUCHE") != 0 &&
      strcmp(commande, "DROITE") != 0
     )
  {
    return;
  }


  // ==========================================================
  // SI UNE ROTATION EST DEJA EN COURS
  //
  // On memorise seulement la nouvelle commande.
  // On ne l'execute pas avant la fin de la rotation.
  // ==========================================================

  if (etat == ROTATION)
  {
    strncpy(
      commandeApresRotation,
      commande,
      sizeof(commandeApresRotation) - 1
    );

    commandeApresRotation[
      sizeof(commandeApresRotation) - 1
    ] = '\0';

    return;
  }


  // ==========================================================
  // SINON EXECUTER DIRECTEMENT
  // ==========================================================

  executerCommande(commande);
}


// ============================================================
// EXECUTER UNE COMMANDE
// ============================================================

void executerCommande(const char* commande)
{
  if (strcmp(commande, "AVANCE") == 0)
  {
    Serial.println("ACK:AVANCE");

    etat = AVANCE_CONTINUE;

    avancer();
  }

  else if (strcmp(commande, "GAUCHE") == 0)
  {
    Serial.println("ACK:GAUCHE");

    demarrerRotationGauche();
  }

  else if (strcmp(commande, "DROITE") == 0)
  {
    Serial.println("ACK:DROITE");

    demarrerRotationDroite();
  }
}


// ============================================================
// ROTATION GAUCHE
// ============================================================

void demarrerRotationGauche()
{
  // Effacer l'ancienne commande memorisee
  commandeApresRotation[0] = '\0';

  Serial.println("BUSY");

  etat = ROTATION;

  debutEtat = millis();

  tournerGauche();
}


// ============================================================
// ROTATION DROITE
// ============================================================

void demarrerRotationDroite()
{
  // Effacer l'ancienne commande memorisee
  commandeApresRotation[0] = '\0';

  Serial.println("BUSY");

  etat = ROTATION;

  debutEtat = millis();

  tournerDroite();
}


// ============================================================
// GESTION DE LA ROTATION
// ============================================================

void gererMouvement()
{
  if (etat == ROTATION)
  {
    if (millis() - debutEtat >= tempsRotation25)
    {
      etat = ARRET_LIBRE;

      Serial.println("TERMINE");


      // ======================================================
      // UNE NOUVELLE COMMANDE A ETE RECUE PENDANT LA ROTATION
      // ======================================================

      if (commandeApresRotation[0] != '\0')
      {
        char commandeTemp[16];

        strncpy(
          commandeTemp,
          commandeApresRotation,
          sizeof(commandeTemp) - 1
        );

        commandeTemp[
          sizeof(commandeTemp) - 1
        ] = '\0';

        // Effacer avant d'executer
        commandeApresRotation[0] = '\0';

        executerCommande(commandeTemp);
      }

      // ======================================================
      // AUCUNE NOUVELLE COMMANDE
      // ======================================================

      else
      {
        arreterMoteurs();
      }
    }
  }
}


// ============================================================
// AVANCER
// ============================================================

void avancer()
{
  analogWrite(ENA, vitesseAvanceDroite);
  analogWrite(ENB, vitesseAvanceGauche);

  // Cote droit : avant
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Cote gauche : avant
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}


// ============================================================
// TOURNER A GAUCHE
// ============================================================

void tournerGauche()
{
  analogWrite(ENA, vitesseRotation);
  analogWrite(ENB, vitesseRotation);

  // Cote droit avance
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Cote gauche recule
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}


// ============================================================
// TOURNER A DROITE
// ============================================================

void tournerDroite()
{
  analogWrite(ENA, vitesseRotation);
  analogWrite(ENB, vitesseRotation);

  // Cote droit recule
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  // Cote gauche avance
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}


// ============================================================
// ARRET DES MOTEURS
// ============================================================

void arreterMoteurs()
{
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
