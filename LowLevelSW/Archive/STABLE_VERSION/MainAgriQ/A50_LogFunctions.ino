/*

void SDinit() {
  // Inizializzo la scheda SD
  //Serial.print("Inizializzazione SD card: \t...\t");
  if (!SD.begin(BUILTIN_SDCARD)){
    Serial.println("SD Failed Initialization!");
    return;
    }
  //Serial.println("inizializzazione RIUSCITA.");
  // Controllo ci sia il file logFile altrimenti lo creo
  if (SD.exists(logFile))
  {
    //Serial.println(logName + " esiste;");
  }
  else
  {
    Serial.println(logName + " DOES NOT exist");
    // open a new file and immediately close it:
    Serial.println("Creating " + logName);
    myLOG = SD.open(logFile, FILE_WRITE);
    myLOG.close();
    while (!SD.exists(logFile)) {;}
  }
}

void SDlog(String dataString) {
  myLOG = SD.open(logFile, FILE_WRITE);

  if(myLOG){
    myLOG.println(dataString);
    myLOG.close();
  }
  else {
    Serial.println("Error opening the log file " + logName);
  }
}

*/
