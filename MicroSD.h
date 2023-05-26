#ifndef MicroSD_h
#define MicroSD_h
/* Librerías fabricante necesarias para utilizar la MicroSD */
#include <SD.h>
#include <SPI.h>

#define MICROSD_PIN 5    /* Pin de control MicroSD */  
File MicroSD_File;       /* Archivo para escribir  o leer en la MicroSD */
class MicroSD  {
  // VARIABLES
  public:
    uint8_t contador = 0;      /* */
    String filename = "/TestMicroSD.txt";   /* Nombre de archivo que incluye "/" que indica la ubicación en raíz */
  // MÉTODOS
  public:
   void MicroSD_init ( void );
   void SaveFile ( void );
   void ReadFile ( void );

};

void MicroSD::MicroSD_init ( void ){

   while ( !SD.begin ( MICROSD_PIN ) ) {
        Serial.println ( F ( "Falló la inicialización del módulo MicroSD"  ));
        delay(1000);
    }
    Serial.println ( F ( "La librería MicroSD ha inicializado con éxito" ) );
}

void MicroSD :: SaveFile ( void ) {
    
    // Abrir o crear en caso de que no exista.
    MicroSD_File = SD.open ( filename, FILE_APPEND );
    if ( MicroSD_File ) {

        // Almacenar JSON en el archivo.
        //serializeJson(*doc, JSON);
        MicroSD_File.print ( F ( "Contador en: " ) );
        MicroSD_File.println ( contador );
        contador++;
        MicroSD_File.close ( );
      
    } else {
        // if the file didn't open, print an error:
        Serial.print ( F ( "Error opening " ) );
        Serial.println ( filename );
 
       
    }
    Serial.println ( "Verificando si existe el archivo: ");
    if ( SD.exists ( filename ) ) {
    Serial.println ( filename );
    Serial.println( " existe" );
  } else {
        Serial.println ( filename );
    Serial.println( " no existe" );
  }
     // Esperar un segundo entre lecturas
      delay ( 1000 );
}
void MicroSD :: ReadFile ( void ) {

    //String filename = "/TestMicroSD.txt";
    // re-open the file for reading:
  MicroSD_File = SD.open( filename );
  if ( MicroSD_File ) {
    Serial.println( "Archivo:" );

    // read from the file until there's nothing else in it:
    while ( MicroSD_File.available ( ) ) {

      Serial.write ( MicroSD_File.read ( ) );

    }
    // close the file:
    MicroSD_File.close ( );
  } else {
    // if the file didn't open, print an error:
    Serial.println ( "Error abriendo el archivo" );
  }

}

#endif