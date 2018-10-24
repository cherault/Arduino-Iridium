#include <PString.h>                                                                                                                                                                                                                                                                                                                                                                                                    #include <PString.h>
#include <IridiumSBD.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

//--temps entre 2 envoie -> 6H = 6*3600--//
//---------------------------------------//
#define BEACON_INTERVAL 21600 

//--port tserie Sat--//
//-------------------//
#define ROCKBLOCK_RX_PIN 9
#define ROCKBLOCK_TX_PIN 8
#define ROCKBLOCK_SLEEP_PIN 10
#define ROCKBLOCK_BAUD 19200

//--port serie GPS--//
//------------------//
#define GPS_RX_PIN 2
#define GPS_TX_PIN 3
#define GPS_BAUD 9600 

//--port serie console--//
//----------------------//
#define CONSOLE_BAUD 115200

SoftwareSerial ssIridium(ROCKBLOCK_RX_PIN, ROCKBLOCK_TX_PIN); //config. port serie SAT
SoftwareSerial ssGPS(GPS_RX_PIN, GPS_TX_PIN); //config. port serie GPS

IridiumSBD isbd(ssIridium, ROCKBLOCK_SLEEP_PIN);
TinyGPSPlus tinygps;

void setup()
{
  //--active port serie console--//
  //-----------------------------//
  Serial.begin(CONSOLE_BAUD);

  //--config. iridium--//
  //-------------------//
  isbd.attachConsole(Serial);
  isbd.attachDiags(Serial);
  isbd.setPowerProfile(1);
}

void loop()
{
  bool fixFound = false;
  unsigned long loopStartTime = millis();
  
  //--lecture tension batterie--//
  //----------------------------//
  int voltageBat = analogRead(A0); 
  float voltage = voltageBat * (5.0 / 1023.0); // CAN 10 bits
  float readBat = voltage * 3.22; // lecture tension batterie reelle

  //--active port serie SAT & GPS--//
  //-------------------------------//
  ssIridium.begin(ROCKBLOCK_BAUD);
  ssGPS.begin(GPS_BAUD);

  //--reset GPS & ecoute traffic GPS--//
  //----------------------------------//
  Serial.println("Debut ecoute traffic GPS...");
  tinygps = TinyGPSPlus();
  ssGPS.listen();

  //--cherche signal GPS toutes les 7mn--//
  //-------------------------------------//
  for (unsigned long now = millis(); !fixFound && millis() - now < 7UL * 60UL * 1000UL;)
    if (ssGPS.available())
    {
      tinygps.encode(ssGPS.read());
      fixFound = tinygps.location.isValid() && tinygps.date.isValid() &&
        tinygps.time.isValid() && tinygps.altitude.isValid();
    }

  Serial.println(fixFound ? F("GPS trouve !") : F("Pas de GPS trouve"));

  //--active SAT et debut communication--//
  //-------------------------------------//
  Serial.println("Debut de comm. avec Iridium...");
  ssIridium.listen();
  if (isbd.begin() == ISBD_SUCCESS)
  {
    char outBuffer[60]; // on essaye de garder un message court !
    if (fixFound)
    {
      //--creation du message--//
      //-----------------------//
      //sprintf(outBuffer, "%d%02d%02d%02d%02d%02d,",
      sprintf(outBuffer, "%d%02d%02d%02d%02d,",
        tinygps.date.year(), tinygps.date.month(), tinygps.date.day(), // date
        tinygps.time.hour(), tinygps.time.minute(), tinygps.time.second()); // heure 
      int len = strlen(outBuffer);
      PString str(outBuffer, sizeof(outBuffer) - len);
      str.print(tinygps.location.lat(), 6); // lat GPS
      str.print(",");
      str.print(tinygps.location.lng(), 6); // long GPS
      //str.print(",");
      //str.print(tinygps.altitude.meters());
      str.print(",");
      str.print(tinygps.speed.knots(), 1); // vitesse (noeuds)
      str.print(",");
      str.print(readBat, 1);
      delay(100); // pour stabilite de lecture
    }
    else
    {
      sprintf(outBuffer, "Pas de GPS trouve !");
    }

    //--envoi du message--//
    //--------------------//
    Serial.print("Transmission du message: ");
    Serial.println(outBuffer);
    isbd.sendSBDText(outBuffer);
  }

  //--mode sommeil -> envoi outes les 6H--//
  //--------------------------------------//
  Serial.println("Entre en mode sommeil pendant 6H...");
  isbd.sleep();
  ssIridium.end();
  ssGPS.end();
  int elapsedSeconds = (int)((millis() - loopStartTime) / 1000);
  while (elapsedSeconds++ < BEACON_INTERVAL)
    delay(1000);
}
