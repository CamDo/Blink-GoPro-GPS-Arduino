/* 
14th August 2017
CamDo Solutions Inc.
Arduino sketch to activate the GoPro camera based on the distance travelled
calculated from an attached GPS.
Requires the CamDo Blink time lapse controller, Arduino and external GPS module.
See cam-do.com/blog for details.

Uses the TinyGPS libary - make sure to use V1.3 or later.
as versions 1.22 and earlier gave lat/long in 100,000ths of a degree. V1.3 reports in millionths of a degree.
Careful not to use the TinyGPS++ library.
*/

#include <TinyGPS.h>
#include <LiquidCrystal.h>

/******** LCD driver pins ********
note these pins are different to the defaults in the Arduino LiquidCrystal library examples.
you may need to change back to the Arduino defaults depending on the model LCD 16x2 display you get.
**********************************/
// uncomment the following line if you are having trouble with your display (Arduino defaults) 
// const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;

// comment out the following line if you go back to the Arduino defaults above.
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7); // initialise the display library

/* LCD variable */
const int led =  LED_BUILTIN;  //typically pin13 but constant means we don't have to worry about that
boolean displayOn = 1;
int buttonVoltage = 0;
int buttonPressed = 0;
/* the backlight is controlled by PWM on D10. Therefore set D10 as a PWM output, then */
/*analogWrite (backLight, fadeValue); */
int backLight = 10;
int fadeValue = 125;
int noPhotos = 0; // stores the number of photos triggered


/* GPS parameters */
TinyGPS gps;
long lat, lon;
float fLat, fLon, otherfLat, otherfLon, fkmph = 0;

unsigned long fix_age, time, date, speed, course;
int year;
byte month, day, hour, minute, second, hundredths;
unsigned long chars;
unsigned short sentences, failed_checksum;
int distanceInMeters = 0;
float tempDistance = 0;
int triggerDistance = 50;                   // distance in metres between photos. Set to a low number for testing (50m). Can be changed here and also via LCD buttons (left and right)
const float pauseSpeed = 2.5;               // in kmph - below which the distance measured is paused to help remove some of the GPS noise when standing still.
const long timeInterval = 5000;             // 5 sec time interval for smoothing GPS data (milliseconds)
const long displayInterval = 1000;          // 1 sec time to show each message on the LCD for before the next message rolls through

int deg;
int min1;
int min2;

/* CamDo Blink interface pins */
const int cameraTriggerPin = 2;             // connected to digital pin 2
const int cameraStatusPin = 3;              // connected to digital pin 3
int cameraState = LOW;                      // cameraState used to store camera status (on/off). Not being used in this project.

unsigned long previousMillis = 0;           // will store last time update
unsigned long currentMillis = 0;
unsigned long previousMillis1 = 0;          // will store last time distance was updated
unsigned long currentMillis1 = 0;

static const float SYDNEY_LAT = -33.80364, SYDNEY_LON = 151.289797;
boolean firstBoot;

/* Initialisation */
void setup()
{
  pinMode(cameraTriggerPin, OUTPUT);        // set cameraTriggerPin as an output
  pinMode(cameraStatusPin, INPUT);          // sets cameraStatusPin as an input   
  pinMode(led, OUTPUT);                     // sets the led as an output
  Serial.begin(9600);                       // sets the GPS baud rate.
    
  //updateGPSSettings();                    // update refresh rate if desired - default is 1 Hz so not generally necessary
  firstBoot = true;
  tempDistance = 0;
  previousMillis = millis();                // store current time for display timing
  previousMillis1 = millis();               // store current time for GPS distance smooting
  startLCD();
  

}
/*********************/
/* Main program loop */
/*********************/
void loop()
{
  bool newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
  while (Serial.available())
  {
    digitalWrite(led, HIGH);
    int c = Serial.read();                   // Read the GPS data
    if (gps.encode(c))                       // Did a new valid sentence come in?
    {
      newData = true;
    }

  }
  }
  digitalWrite(led, LOW);
  

  
  if (newData)
  {
      gps.get_position(&lat, &lon, &fix_age);             // retrieves +/- lat/long in millionths of a degree
      gps.f_get_position(&fLat, &fLon, &fix_age);         // the float version which helps for distance calcs
      // note - make sure to be using V1.3 or greater of TinyGPS. Earlier versions return in 100,000th's of a degree.
      gps.get_datetime(&date, &time, &fix_age);           // time in hhmmsscc, date in ddmmyy
      

      if (fix_age == TinyGPS::GPS_INVALID_AGE){           // check the data status
        noGPSFix();
      }
      if (fix_age > 5000){
        staleGPSData();
      }
    
     fkmph = gps.f_speed_kmph();
     
     // Uncomment the following if you want to use the date / time parameters for anything
     // Date/time cracking - convert the date time into easier to use variables. 
     // gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &fix_age);  
     
     // check if distance threshold has been met and trigger camera as appropriate
    

    // update the time
    currentMillis = millis();

    // we check timing and display the messages on the LCD screen.
    // need to have initial threshold around 3000 ms (ie 3 x displayInterval) to allow GPS time for processing in loop above.
    // we measure the time and take an action rather than just setting a delay for each LCD message to avoid blocking the processor.
    
    if ((currentMillis - previousMillis) <= 3*displayInterval) {
         if (checkDistance()){
          triggerCamera();
          }
          printDistancePhotos();
    }
    else if ((currentMillis - previousMillis) <= 4*displayInterval) {
        printLat();     // update the LCD screen with lat/long position. Not required if you don't have an LCD.
        printLon();
    }
    else if ((currentMillis - previousMillis) <= 5*displayInterval) {
        printSatellitesSpeed();  // As above, have to balance update of info on screen with buttonpress responsiveness.
           
    }
    // keep adding else if statements in here and increase the displayInterval multiplier for each extra if statement.
    
    else {
      // we've dealt with all of the displays and reset our counter.
      previousMillis = millis();
    }
    
     
  }
  else {
    noGPSFix();
       
  }
       
  // check if buttons pressed on LCD display and do something.
  // see checkButtonPress() routine for explanations.
  checkButtonPress();
  
  
  
} // end loop



/*******************************/
/******* Routines **************/
/*******************************/

// routine to update the UBLOX baud rate, etc. Should not have to use this.
// not called by default.
void updateGPSSettings(){
 const char UBLOX_INIT[] PROGMEM = {
  // Refer pg 14 / Chp 8 in manual for available settings.
  // Reset to manufacturer defaults
  0xB5,0x62,0x06,0x09,0x0D,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x07,0x1F,0x9E,
  // Update Rate to 10 Hz
  0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12
  };

  // write to GPS module
  for(int i = 0; i < sizeof(UBLOX_INIT); i++){
    Serial.write( pgm_read_byte(UBLOX_INIT+i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }

}

/* Print the latitude */
void printLat(){                      
  
  deg=lat/1000000;
  min1=(lat/10000)%100;
  min1=abs(min1);                 // take the absolute value since deg is signed. Note, do not combine mathematical operations inside abs() function.
  min2=lat%10000;
  min2 = abs(min2);               // take the absolute value since deg is signed. 

  lcd.setCursor(0,0);             // set the LCD cursor position 
  lcd.print("LAT:");              
  lcd.print(deg);
  lcd.print(".");
  lcd.print(min1);
  lcd.print(min2);
  lcd.write(0xDF);                // write the degrees symbol
  lcd.write("  ");
}

/* Print the longitude */
void printLon(){                  
  
  deg=lon/1000000;
  min1=lon/10000%100;
  min1=abs(min1);                 // take the absolute value since deg is signed. 
  min2=lon%10000;
  min2=abs(min2);                 // take the absolute value since deg is signed.

  lcd.setCursor(0,1);              // set the LCD cursor position 
  lcd.print("LON:");              
  lcd.print(deg);
  lcd.print(".");
  lcd.print(min1);
  lcd.print(min2);
  lcd.write(0xDF);                  // write the degrees symbol
  lcd.write("  ");
}

/* report to the user there is no GPS fix */
void noGPSFix(){                  
  
  lcd.setCursor(0,0);              // set the LCD cursor position
  lcd.print("Waiting:          ");              
  lcd.setCursor(0,1);              // set the LCD cursor position 
  lcd.print("No GPS fix yet!  "); 
     
}

/* Print number of satellites and current speed in km/h */
void  printSatellitesSpeed(){                  
  
  lcd.setCursor(0,0);              // set the LCD cursor position
  lcd.print("Satellites: ");              
  lcd.print(gps.satellites()); 
  lcd.print("              ");
  lcd.setCursor(0,1);              // set the LCD cursor position 
  lcd.print("Speed km/h: ");      
  lcd.print(fkmph);  
  lcd.print("              ");
     
}

/* report to the user the GPS NMEA data may be stale */
void staleGPSData(){
  
  lcd.setCursor(0,0);              // set the LCD cursor position 
  lcd.print("Warning:Possible");      
  lcd.setCursor(0,1);              // set the LCD cursor position 
  lcd.print("!  Stale data  !");              
        
  
}

/* report the time in seconds since the system booted on the LCD */
void printTime () {
 
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print(millis() / 1000);
}

/* report the current speed on the LCD */
void printSpeed () {
 
  lcd.setCursor(0,0);              // set the LCD cursor position 
  lcd.print("Speed km/h:     ");      
  lcd.setCursor(0,1);              // set the LCD cursor position 
  lcd.print(fkmph);              
  lcd.print("                      ");
  

// other speed / position / altitude commands for reference
// returns +/- latitude/longitude in degrees
//  gps.f_get_position(&fLat, &fLon, &fix_age);
//  float falt = gps.f_altitude(); // +/- altitude in meters
//  float fc = gps.f_course(); // course in degrees
//  float fk = gps.f_speed_knots(); // speed in knots
//  float fmph = gps.f_speed_mph(); // speed in miles/hr
//  float fmps = gps.f_speed_mps(); // speed in m/sec
//  float fkmph = gps.f_speed_kmph(); // speed in km/hr
    
}

/* check the distance travelled against our preset and see if we need to trigger the camera */
boolean checkDistance() {
  // we need to check the distance travelled from previous trigger location
  // it is actual distance travelled (recorded every 5 seconds), not line of sight.
  // return true if threshold reached.
  
  
  // check to see if we have just booted up and store coords.
  if (firstBoot){
    otherfLat  = fLat;
    otherfLon  = fLon;
    distanceInMeters = 0;
    firstBoot = false;
  }

  // only update distance every few seconds to smooth out GPS a little. 
  // can be set in global variables at top of sketch (default 5 secs)

  currentMillis1 = millis();

  if ((currentMillis1 - previousMillis1) >= timeInterval) {
    // save the new time interval
    previousMillis1 = currentMillis1;

//    gps.f_get_position(&fLat, &fLon, &fix_age); // the float version which helps for distance calcs
    
    // update distance travelled
    tempDistance = gps.distance_between(fLat, fLon, otherfLat, otherfLon);
    
    // remove the spurious nature of GPS if standing still
    // if speed less than the pause speed and distance less than 3m then skip
    // can comment out the distance / speed check when testing so you can test on your bench without moving around!
    if(tempDistance > 3 && fkmph > pauseSpeed)
    {
      distanceInMeters = distanceInMeters + tempDistance;
    }
      
    if (distanceInMeters >= triggerDistance) {
      // reset distance travelled and update coords
      distanceInMeters = 0;
      otherfLat  = fLat;
      otherfLon  = fLon;
      return true;
    }
    else
      return false;
  
  }
  else
    return false;
    
  
}

/* display the distance travelled on the LCD*/
void printDistancePhotos (){
  lcd.clear();
  lcd.setCursor(0,0);              // set the LCD cursor position 
  lcd.print("Dist(m): ");      
  lcd.print(distanceInMeters);
  lcd.print("                      ");   
  lcd.setCursor(0,1);              // set the LCD cursor position 
  lcd.print("Photos: ");               
  lcd.print(noPhotos); 
  lcd.print("                      ");
  //delay(1000);
}


/* trigger the GoPro using CamDo Blink controller*/
void triggerCamera () {
  digitalWrite(cameraTriggerPin, HIGH);           // send a high signal to Blink to trigger the camera
  delay(1000);                                    // hold the trigger for 1000 milliseconds
  digitalWrite(cameraTriggerPin, LOW);            // send a low signal to Blink to finish trigger
  cameraState = digitalRead(cameraStatusPin);     // read the status of the camera from Blink. Not doing anything with it at this stage.
  noPhotos++;                                     // increment the number of photos taken
}


/* initialises the LCD and prints a welcome message */
void startLCD () {
  
  lcd.begin(16, 2);               // start the LCD library, setup with 16 x 2 characters
  lcd.setCursor(0,0);             // set the LCD cursor position (Column 0, Line 0)
  lcd.print("CamDo Solutions");   // print to LCD
  delay(200);
 
  lcd.setCursor(0,1);             // set the LCD cursor position (Column 0, Line 1 (second row))
  char* welcome = "GPS PhotoTrigger";
  for (int cnt = 0; cnt <= 16; cnt++) {
    lcd.print(welcome[cnt]);
    delay(100);
  }
 
  delay (2000);
  lcd.clear();
  
}

/* check the integer length to help line text up on the LCD */
/* NOT BEING USED */
int checkIntLen(int someValue) {

  int valLen = 0;

  if(someValue > 9999)
    valLen = 5;
  else if(someValue > 999)
    valLen = 4;
  else if(someValue > 99)
    valLen = 3;
  else if(someValue > 9)
    valLen = 2;
  else
    valLen = 1;
    
    return valLen;
}


/* Check if one of the LCD buttons has been pressed and do something */
/* thanks to the DX.com forum for the outline */
/* http://club.dx.com/forums/Forums.dx/threadid.1194279 */
/* the backlight is controlled by PWM on D10 (var backLight). Set D10 as a PWM output, then */
/* analogWrite (backLight, fadeValue); */

void checkButtonPress() {
  // pressing UP or DOWN will fade the display backLight up / down.
  // pressing SELECT will turn the screen ON / OFF.
  // pressing LEFT or RIGHT will increase or decrease the camera trigger distance.
  // distance is in metres.
  
  buttonVoltage = analogRead(0);
  buttonPressed = buttonVoltage >> 7; //this removes least significant bits to get more stable reads.
  
  switch (buttonPressed)
  {
  case 0: // RIGHT button. 
   //Increase triggerDistance by varying degrees depending on current setting.
    lcd.setCursor(0,0);  
    lcd.print ("Distance(m):   ");
    lcd.setCursor(0,1);
    lcd.print(triggerDistance);
    lcd.print("               ");
    delay(100);
    if (triggerDistance >= 800)
    {
      triggerDistance = triggerDistance + 100;
    }
    else if(triggerDistance > 300)
    {
      triggerDistance = triggerDistance + 50;
    }
    else
    {
    triggerDistance = triggerDistance + 20; //Decrease triggerDistance by 20m at a time.
    }
    
    lcd.setCursor(0,0);  
    lcd.print ("Distance(m):   ");
    lcd.setCursor(0,1);
    lcd.print(triggerDistance);
    lcd.print("               ");
    
    //lcd.scrollDisplayRight() ;
    break;
  case 1: // UP button
    fadeValue = fadeValue +5;
    if (fadeValue >= 254)
    {
      fadeValue = 255;
    }
    analogWrite (backLight, fadeValue);
    
    break;
  case 2: // DOWN button. 
    fadeValue = fadeValue -5;
    if (fadeValue <= 1)
    {
      fadeValue = 0;
    }
    analogWrite (backLight, fadeValue);
   
    break;
  case 3: // LEFT button. 
    lcd.setCursor(0,0);  
    lcd.print ("Distance(m):   ");
    lcd.setCursor(0,1);
    lcd.print(triggerDistance);
    lcd.print("               ");
    delay(100);
    // make the distance scroll a bit quicker when numbers are larger
    if (triggerDistance >= 800)
    {
      triggerDistance = triggerDistance - 100;
    }
    else if(triggerDistance > 300)
    {
      triggerDistance = triggerDistance - 50;
    }
    else
    {
    triggerDistance = triggerDistance - 20; //Decrease triggerDistance by 20m at a time.
    }
    
    lcd.setCursor(0,0);
    lcd.print ("Distance(m):    ");
    lcd.setCursor(0,1);
     if (triggerDistance <= 1)
    {
      triggerDistance = 0;
      lcd.print("!!  Disabled  !!");
    }
    else
    {
    lcd.print(triggerDistance);
    lcd.print("               ");
    }
    //lcd.scrollDisplayRight() ;
    break;
  case 5: // SELECT button
    lcd.print ("Select");
    if (displayOn) 
    {
      displayOn=false; 
      lcd.noDisplay();
      analogWrite (backLight, 0);
    }
    else 
    {
      displayOn=true; 
      lcd.display();
      analogWrite (backLight, fadeValue);
    }
    delay(500);
    break;
  case 7: // no button press
    //lcd.print ("None ");
    break;
  }
  delay(100); // delay for button press usability
}


