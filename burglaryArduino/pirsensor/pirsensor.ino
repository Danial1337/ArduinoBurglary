// This #include statement was automatically added by the Particle IDE.
#include <OneWire.h> 
#include <DS18.h> //library for DS18-sensor

DS18 sensor(D6);
const int pirSensorPin = D0;//deklarasjon av pir-sensor pin
const int buzzerPin = D1; //deklarasjon av buzzer-pin
const int redPin = D2;
const int greenPin = D3;
const int bluePin = D4;
const int buttonPin = D5; //deklarasjon av button-pin
const int ledPin = D7;   // deklarasjon av led-pinne(innebygde led-lyset)

int pirState = LOW;             // Vi starter med utgangspunkt i at ingen bevegelse er detected
int val = 0;                    // Variabel for å lese pirsensorPin, starter med 0

double temperature;
String tempAsString;

void setup() {
 pinMode(ledPin, OUTPUT);      // deklarere LED(D7) som output
 pinMode(pirSensorPin, INPUT);     // deklarere sensorPin(D0) som input
  
 pinMode(redPin, OUTPUT);
 pinMode(greenPin, OUTPUT);
 pinMode(bluePin, OUTPUT);
 
 pinMode(buttonPin, INPUT); //deklarere buttonPin(D5) som input
 
 
 Particle.syncTime(); //Synctime() for å synce tiden med cloud-tiden til Particle
 
 Serial.begin(9600);
  
 Particle.function("testBuzzer", testBuzzer); //"exposer" xxxxx-funksjon til particle-cloud(appen, i mitt tilfelle). 
 Particle.function("testLed", testLed);
 
 Particle.variable("temperature", temperature); //"exposer" temp-variabel til particle-cloud, og kan ses i appen.
 Particle.variable("temperatureString", tempAsString);//"exposer" tempAsString-variabel til particle-cloud, og kan ses i particle-appen under fanen "variables"
 

}


int testBuzzer(String input){ //en "test" som kan sjekke om buzzer'n fungerer ved hjelp av functions-fanen appen.
    if(input == "on" || input == "ON" || input =="On" || input == "oN"){
        tone(buzzerPin, 2000, 50000); //3 parametre: 1) pin 2) hvilket frekvens 3)duration i ms
        Serial.println("Buzzer er skrudd på!");
        return 1;
       
    } else if (input == "off"){
        digitalWrite(buzzerPin, LOW);
        Serial.println("Buzzer er avskrudd!");
        return 1;
    }
}


int testLed(String input){
    if(input == "red" || input == "on" || input == "ON" || input =="On" || input == "oN"){
        digitalWrite(D2, HIGH);
        Serial.println("Rødt Lys skrudd på!");
    } else if (input == "off"){
        digitalWrite(D2, LOW);
        Serial.println("Rødt lys avskrudd!");
    }
}
 
void loop(){
  val = digitalRead(pirSensorPin);  // read input value
 int buttonState = digitalRead(buttonPin);
 temperature = sensor.celsius();
 
 if(sensor.read()){ //If-else for å sjekke om sensor leser, og deretter printe data i både serialmonitor og cloud(som variabel)
        temperature = sensor.celsius();
        tempAsString=String(temperature);
        
        Serial.print(temperature);
        Serial.print("*C");
        Serial.println(" ");
       
        Particle.variable("temperature", temperature);
        Particle.variable("temperatureString", tempAsString);
    };
 
 if(buttonState == LOW){  //IF-else for å sjekke om knappen er trykket, hvis den er trykket ønsker vi å publishe event med temperturmåling og vise i Serial
        if(sensor.read()){
            
            Serial.println("***.........KNAPP TRYKKET.........***");
            Serial.println("***...AKTIVERER PARTICLE.PUBLISH EVENT...*****");
            Particle.publish("temperature", String(sensor.celsius()), PRIVATE);// event
    
            temperature = sensor.celsius();
    
            Serial.println("...");
            Serial.println("Temperaturen er:");
            Serial.print(temperature);
            Serial.print("*C");
           Serial.println("  ");

           delay(10000); // Delay i 10 sekunder etter at knappen er trukket ned
          }
    }else{} //Gjør absolutt ingen ting om knappen ikke er LOW(ikke trukket ned);
 
 
  if (val == HIGH) { //Val er deklarert til å lese pirSensorPin, og dersom den er HIGH så skal innbrudd varsles.
  
    digitalWrite(ledPin, HIGH);  // skru på LED (innebygde LED på photon article)
    Serial.println("Innbrudd har skjedd!!!");
    Serial.println("  ");
    
    Serial.print("Innbruddet skjedde:");
    Serial.print(Time.timeStr());
    Serial.println(" ");
    
    tone(buzzerPin, 2300, 15000);//3 parametre: 1) pin 2) hvilket frekvens 3)duration i ms
    setColor(255, 0, 0);
   
   
        if(val == HIGH){
          Particle.publish("BurglaryEvent");
          Serial.println("Sender event BurglaryEvent og venter i 10sek.....");
          Serial.println(" .");
          delay(10000);
          setColor(0,0,0);
         }
   
        if (pirState == LOW) { // pirState er forhåndsbestemt til å være low på dette tidspunktet da vi har deklarert den til LOW, derfor vil denne "if"-statement være "true" i første omgang.
           
           Serial.println("Innbrudd har skjedd!!! Avslutter SCAN!(testNote: etter å ha ventet i 10sek etter BurglaryEvent skjer dette)" );
           Serial.println(".");
           
           pirState = HIGH; //Da pirState er forhåndsbestemt til å være LOW, så setter vi den til HIGH når alarmen har gått. at den blir "HIGH" triggrer "else"-delen av koden
          }
          
 
  } else {
    digitalWrite(ledPin, LOW); // Skru av D7-led lyset
    if (pirState == HIGH){ 
    
      Serial.println("Scanning slutt. Restarter Scan -->");
      Serial.println("...");
      Serial.println(" ....");
      Serial.println(" ......");
     
     pirState = LOW; //setter pirState til low, slik at de tidligere
                 //if-statements evalueres til true når loop() kjøres igjen
                   
    }
  }
}

void setColor(int redLight, int greenLight, int blueLight){
    analogWrite(redPin, redLight);
    analogWrite(greenPin, greenLight);
    analogWrite(bluePin, blueLight);
}
