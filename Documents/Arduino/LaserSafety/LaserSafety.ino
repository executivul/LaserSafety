
/*-----( Import needed libraries )-----*/
#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>

/*-----( Declare Constants )-----*/
//global values
#define version "v0.9"
#define LCD_UPDATE_DELAY 500
#define ALERT_TEMP 28
#define ERROR_TEMP 29

//pindefs
#define DS_PIN 3
#define FLOW_PIN 2
#define LED_R 9
#define LED_G 10
#define LED_B 12
#define RELAY_1 4
#define RELAY_2 5
#define ACTIVE LOW
#define INACTIVE HIGH
#define DS_READ_DELAY 1000

/*-----( Declare objects )-----*/
OneWire  ds(DS_PIN);
// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address


/*-----( Declare Variables )-----*/
//hardcoded addresses for the 3 sensors used
const byte addr_out[8]={0x28,0xFF,0xA8,0x1F,0x92,0x15,0x01,0xCD};
const byte addr_in[8]={0x28,0xFF,0x73,0x81,0x91,0x15,0x04,0xCC};
const byte addr_tank[8]={0x28,0xFF,0x6F,0xA3,0x91,0x15,0x04,0xE6};
uint16_t in_millis=0;
uint16_t out_millis=0;
uint16_t tank_millis=0;
uint16_t lcd_millis=0;
bool in_read=20;
bool out_read=20;
bool tank_read=20;
float in_temp,out_temp,tank_temp;
uint8_t error=0;

/*----( SETUP: RUNS ONCE )----*/
void setup()   
{
  Serial.begin(115200);  // Used to type in characters

  pinMode(LED_R,OUTPUT);
  pinMode(LED_G,OUTPUT);
  pinMode(LED_B,OUTPUT);
  setLed(0,0,255);

  pinMode(RELAY_1,OUTPUT);
  pinMode(RELAY_2,OUTPUT);
  digitalWrite(RELAY_1,INACTIVE);
  digitalWrite(RELAY_2,INACTIVE);
  

  lcd.begin(20,4);         // initialize the lcd for 20 chars 4 lines, turn on backlight
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0); //Start at character 4 on line 0
  lcd.print("Laser Safety System");
  lcd.setCursor(4,1);
  lcd.print("@ Mircea 06.2016");
  lcd.setCursor(10,3);
  lcd.print(version);
  delay(1000);
  lcd.clear();

}/*--(end setup )---*/


void loop()   /*----( LOOP: RUNS CONSTANTLY )----*/
{
    byte data[12];
    byte i;
    
    uint16_t now_millis=millis();
    if ((now_millis - in_millis > DS_READ_DELAY)&&(in_read==0)){
      ds.reset();
      ds.select(addr_in);
      ds.write(0x44, 1);        // start conversion, with parasite power on at the end
      in_read = 1;
      in_millis=now_millis;
      Serial.print("Millis:");Serial.print(now_millis);Serial.println(" Request IN conv ");
      delay(2);
    } 
    if ((now_millis - out_millis > DS_READ_DELAY)&&(out_read==0)){
      ds.reset();
      ds.select(addr_out);
      ds.write(0x44, 1);        // start conversion, with parasite power on at the end
      out_read = 1;
      out_millis=now_millis;
      Serial.print("Millis:");Serial.print(now_millis);Serial.println(" Request OUT conv ");
      delay(2);
    } 
    if ((now_millis - tank_millis > DS_READ_DELAY)&&(tank_read==0)){
      ds.reset();
      ds.select(addr_tank);
      ds.write(0x44, 1);        // start conversion, with parasite power on at the end
      tank_read = 1;
      tank_millis=now_millis;
      Serial.print("Millis:");Serial.print(now_millis);Serial.println(" Request TANK conv ");
      delay(2);
    } 
    
    if ((now_millis - in_millis > DS_READ_DELAY)&&(in_read==1)){
      in_read=0;
      in_millis=now_millis;
      ds.reset();
      ds.select(addr_in);    
      ds.write(0xBE);         // Read Scratchpad
      for ( i = 0; i < 9; i++) {           // we need 9 bytes
        data[i] = ds.read();
      }
      int16_t raw = (data[1] << 8) | data[0];
      in_temp = (float)raw / 16.0;
      Serial.print("Millis:");Serial.print(now_millis);Serial.print(" IN Read: ");Serial.println(in_temp);
    }
    if ((now_millis - out_millis > DS_READ_DELAY)&&(out_read==1)){
      out_read=0;
      out_millis=now_millis;
      ds.reset();
      ds.select(addr_out);    
      ds.write(0xBE);         // Read Scratchpad
      for ( i = 0; i < 9; i++) {           // we need 9 bytes
        data[i] = ds.read();
      }
      int16_t raw = (data[1] << 8) | data[0];
      out_temp = (float)raw / 16.0;
      Serial.print("Millis:");Serial.print(now_millis);Serial.print(" OUT Read: ");Serial.println(out_temp);
    }
    if ((now_millis - tank_millis > DS_READ_DELAY)&&(tank_read==1)){
      tank_read=0;
      tank_millis=now_millis;
      ds.reset();
      ds.select(addr_tank);    
      ds.write(0xBE);         // Read Scratchpad
      for ( i = 0; i < 9; i++) {           // we need 9 bytes
        data[i] = ds.read();
      }
      int16_t raw = (data[1] << 8) | data[0];
      tank_temp = (float)raw / 16.0;
      Serial.print("Millis:");Serial.print(now_millis);Serial.print(" TANK Read: ");Serial.println(tank_temp);
    }

  if((in_temp>ERROR_TEMP)||(out_temp>ERROR_TEMP)||(tank_temp>ERROR_TEMP)||(in_temp<15)||(out_temp<15)||(tank_temp<15)){
    error=2;
    setLed(55,0,0);
    digitalWrite(RELAY_1,INACTIVE);
  } else if((in_temp>ALERT_TEMP)||(out_temp>ALERT_TEMP)||(tank_temp>ALERT_TEMP)){
    error=1;
    setLed(100,50,0);
    digitalWrite(RELAY_1,ACTIVE);
  } else if((in_temp<ALERT_TEMP)&&(out_temp<ALERT_TEMP)&&(tank_temp<ALERT_TEMP)&&(in_temp>18)&&(out_temp>18)&&(tank_temp>18)){
    error=0;
    setLed(0,155,0);
    digitalWrite(RELAY_1,ACTIVE);
  }
  
  
  if(now_millis - lcd_millis > LCD_UPDATE_DELAY){
    lcd_millis=now_millis;
    if (error == 0){
      lcd.clear();
      lcd.setCursor(0,0); 
      lcd.print("TempIN:");
      lcd.print(in_temp);
      lcd.print("*C ");
      lcd.setCursor(0,1);
      lcd.print("TempOUT:");
      lcd.print(out_temp);
      lcd.print("*C ");
      lcd.setCursor(0,2);
      lcd.print("TempTANK:");
      lcd.print(tank_temp);
      lcd.print("*C ");
      lcd.setCursor(0,3);
      lcd.print("Flow:");
      lcd.print("XXX.XX");
      lcd.print(" l/h ");
    }
    if (error == 1){
      lcd.clear();
      lcd.setCursor(0,0); 
      lcd.print("Temperature ALERT!");
      lcd.setCursor(0,1); 
      lcd.print("TempIN:");
      lcd.print(in_temp);
      lcd.print("*C ");
      lcd.setCursor(0,2);
      lcd.print("TempOUT:");
      lcd.print(out_temp);
      lcd.print("*C ");
      lcd.setCursor(0,3);
      lcd.print("TempTANK:");
      lcd.print(tank_temp);
      lcd.print("*C ");
    }
    if (error == 2){
      lcd.clear();
      lcd.setCursor(0,0); 
      lcd.print("Temperature CRITICAL");
      lcd.setCursor(0,1); 
      lcd.print("TempIN:");
      lcd.print(in_temp);
      lcd.print("*C ");
      lcd.setCursor(0,2);
      lcd.print("TempOUT:");
      lcd.print(out_temp);
      lcd.print("*C ");
      lcd.setCursor(0,3);
      lcd.print("TempTANK:");
      lcd.print(tank_temp);
      lcd.print("*C ");
    }
  }

}/* --(end main loop )-- */

void setLed(uint8_t R, uint8_t G, uint8_t B){
    analogWrite(LED_R,255-R);
    analogWrite(LED_G,255-G);
    analogWrite(LED_B,255-B);
}

/* ( THE END ) */

