
/*-----( Import needed libraries )-----*/
#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>

/*-----( Declare Constants )-----*/
//global values
#define version "v0.9"
#define LCD_UPDATE_DELAY 500
#define ALERT_MAX_TEMP 28
#define ERROR_MAX_TEMP 29
#define ALERT_MIN_TEMP 18
#define ERROR_MIN_TEMP 15

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
#define FLOW_READ_DELAY 1000
#define FLOW_PULSE_PER_LITER 500

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
uint16_t flow_millis=0;
bool in_read=false;
bool out_read=false;
bool tank_read=false;
float in_temp,out_temp,tank_temp,old_in_temp,old_out_temp,old_tank_temp;
uint8_t error=0;
uint8_t failed_in=0,failed_out=0,failed_tank=0;
float flow;
volatile uint16_t flow_pulse=0;

/*----( SETUP: RUNS ONCE )----*/
void setup()   
{
    uint8_t data[12];
    uint8_t i;
    int16_t raw;

  Serial.begin(115200);  // Used to type in characters

  pinMode(LED_R,OUTPUT);
  pinMode(LED_G,OUTPUT);
  pinMode(LED_B,OUTPUT);
  setLed(0,0,255);

  pinMode(RELAY_1,OUTPUT);
  pinMode(RELAY_2,OUTPUT);
  digitalWrite(RELAY_1,INACTIVE);
  digitalWrite(RELAY_2,INACTIVE);

  pinMode(FLOW_PIN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flow_interrupt, RISING);

  lcd.begin(20,4);         // initialize the lcd for 20 chars 4 lines, turn on backlight
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0); //Start at character 4 on line 0
  lcd.print("Laser Safety System");
  lcd.setCursor(4,1);
  lcd.print("@ Mircea 06.2016");
  lcd.setCursor(10,3);
  lcd.print(version);
  // try to read initial sensor values
  request_conv(addr_in);delay(100);
  request_conv(addr_out);delay(100);
  request_conv(addr_tank);
  //WAIT FOR READ TO COMPLETE 
  delay(1000);
  in_temp=read_sensor(addr_in);
  out_temp=read_sensor(addr_out);
  tank_temp=read_sensor(addr_tank);
  old_in_temp=in_temp;
  old_out_temp=out_temp;
  old_tank_temp=tank_temp;

  lcd.clear();
}/*--(end setup )---*/


void loop()   /*----( LOOP: RUNS CONSTANTLY )----*/
{
    uint16_t now_millis=millis();
    if(now_millis - flow_millis > FLOW_READ_DELAY){
      flow=(((float)flow_pulse/FLOW_PULSE_PER_LITER)/(now_millis-flow_millis))*3600000;
      flow_pulse=0;
      flow_millis=now_millis;
    }
    
    if ((now_millis - in_millis > DS_READ_DELAY)&&(in_read==false)){
      request_conv(addr_in);
      in_read = true;
      in_millis=now_millis;
      Serial.print("Millis:");Serial.print(now_millis);Serial.println(" Request IN conv ");
      delay(2);
    } 
    if ((now_millis - out_millis > DS_READ_DELAY)&&(out_read==false)){
      request_conv(addr_out);
      out_read = true;
      out_millis=now_millis;
      Serial.print("Millis:");Serial.print(now_millis);Serial.println(" Request OUT conv ");
      delay(2);
    } 
    if ((now_millis - tank_millis > DS_READ_DELAY)&&(tank_read==false)){
      request_conv(addr_tank);
      tank_read = true;
      tank_millis=now_millis;
      Serial.print("Millis:");Serial.print(now_millis);Serial.println(" Request TANK conv ");
      delay(2);
    } 
    
    if ((now_millis - in_millis > DS_READ_DELAY)&&(in_read==true)){
      in_read=false;
      in_millis=now_millis;
      in_temp=read_sensor(addr_in);
      if (abs(in_temp-old_in_temp)>10){
        in_temp=old_in_temp;
        failed_in++;
        Serial.print(" IN read error! ");
      } else {
        old_in_temp=in_temp;
        failed_in=0;
        Serial.print("Millis:");Serial.print(now_millis);Serial.print(" IN Read: ");Serial.println(in_temp);
      }
      
    }
    if ((now_millis - out_millis > DS_READ_DELAY)&&(out_read==1)){
      out_read=false;
      out_millis=now_millis;
      out_temp = read_sensor(addr_out);
      if (abs(out_temp-old_out_temp)>10){
        out_temp=old_out_temp;
        failed_out++;
        Serial.print(" OUT read error! ");
      } else {
        old_out_temp=out_temp;
        failed_out=0;
        Serial.print("Millis:");Serial.print(now_millis);Serial.print(" OUT Read: ");Serial.println(out_temp);
      }
    }
    if ((now_millis - tank_millis > DS_READ_DELAY)&&(tank_read==1)){
      tank_read=false;
      tank_millis=now_millis;
      tank_temp = read_sensor(addr_tank);
      if (abs(tank_temp-old_tank_temp)>10){
        tank_temp=old_tank_temp;
        failed_tank++;
        Serial.print(" OUT read error! ");
      } else {
        old_tank_temp=tank_temp;
        failed_tank=0;
        Serial.print("Millis:");Serial.print(now_millis);Serial.print(" TANK Read: ");Serial.println(tank_temp);
      }
    }
  if (flow<100){
    error=4;
    setLed(55,0,0);
    digitalWrite(RELAY_1,INACTIVE);
  } else if((in_temp>ERROR_MAX_TEMP)||(out_temp>ERROR_MAX_TEMP)||(tank_temp>ERROR_MAX_TEMP)||(in_temp<ERROR_MIN_TEMP)||(out_temp<ERROR_MIN_TEMP)||(tank_temp<ERROR_MIN_TEMP)){
    error=2;
    setLed(55,0,0);
    digitalWrite(RELAY_1,INACTIVE);
  } else if((in_temp>ALERT_MAX_TEMP)||(out_temp>ALERT_MAX_TEMP)||(tank_temp>ALERT_MAX_TEMP)||(in_temp<ALERT_MIN_TEMP)||(out_temp<ALERT_MIN_TEMP)||(tank_temp<ALERT_MIN_TEMP)){
    error=1;
    setLed(100,50,0);
    digitalWrite(RELAY_1,ACTIVE);
  } else if((failed_in>0)||(failed_out>0)||(failed_tank>0)){
    error=3;
    setLed(100,50,0);
  } else if((in_temp<ALERT_MAX_TEMP)&&(out_temp<ALERT_MAX_TEMP)&&(tank_temp<ALERT_MAX_TEMP)&&(in_temp>ALERT_MIN_TEMP)&&(out_temp>ALERT_MIN_TEMP)&&(tank_temp>ALERT_MIN_TEMP)){
    error=0;
    setLed(0,155,0);
    digitalWrite(RELAY_1,ACTIVE);
  }
  
  
  if(now_millis - lcd_millis > LCD_UPDATE_DELAY){
    lcd_millis=now_millis;
    update_lcd();
  }

}/* --(end main loop )-- */

void setLed(uint8_t R, uint8_t G, uint8_t B){
    analogWrite(LED_R,255-R);
    analogWrite(LED_G,255-G);
    analogWrite(LED_B,255-B);
}

void request_conv(const uint8_t rom[8]){
  ds.reset();
  ds.select(rom);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
}

float read_sensor(const uint8_t rom[8]){
  uint8_t data[12];
  uint8_t i;
  ds.reset();
  ds.select(rom);    
  ds.write(0xBE);         // Read Scratchpad
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
  int16_t raw = (data[1] << 8) | data[0];
  return (float)raw / 16.0;
}

void update_lcd(){
    if (error == 0){
      lcd.clear();
      lcd.setCursor(0,0);lcd.print("TempIN:");lcd.print(in_temp);lcd.print("*C ");
      lcd.setCursor(0,1);lcd.print("TempOUT:");lcd.print(out_temp);lcd.print("*C ");
      lcd.setCursor(0,2);lcd.print("TempTANK:");lcd.print(tank_temp);lcd.print("*C ");
      lcd.setCursor(0,3);lcd.print("Flow:");lcd.print(flow);lcd.print(" l/h ");
    }
    if (error == 1){
      lcd.clear();
      lcd.setCursor(0,0);lcd.print("Temperature ALERT!");
      lcd.setCursor(0,1);lcd.print("TempIN:");lcd.print(in_temp);lcd.print("*C ");
      lcd.setCursor(0,2);lcd.print("TempOUT:");lcd.print(out_temp);lcd.print("*C ");
      lcd.setCursor(0,3);lcd.print("TempTANK:");lcd.print(tank_temp);lcd.print("*C ");
    }
    if (error == 2){
      lcd.clear();
      lcd.setCursor(0,0);lcd.print("Temperature CRITICAL");
      lcd.setCursor(0,1);lcd.print("TempIN:");lcd.print(in_temp);lcd.print("*C ");
      lcd.setCursor(0,2);lcd.print("TempOUT:");lcd.print(out_temp);lcd.print("*C ");
      lcd.setCursor(0,3);lcd.print("TempTANK:");lcd.print(tank_temp);lcd.print("*C ");
    }
    if (error == 3){
      lcd.clear();
      lcd.setCursor(0,0);lcd.print("Failed read! ");lcd.print(failed_in);lcd.print(" ");lcd.print(failed_out);lcd.print(" ");lcd.print(failed_tank);
      lcd.setCursor(0,1);lcd.print("TempIN:");lcd.print(in_temp);lcd.print("*C ");
      lcd.setCursor(0,2);lcd.print("TempOUT:");lcd.print(out_temp);lcd.print("*C ");
      lcd.setCursor(0,3);lcd.print("TempTANK:");lcd.print(tank_temp);lcd.print("*C ");
    }
    if (error == 4){
      lcd.clear();
      lcd.setCursor(0,0);lcd.print("FLOW LOW ");lcd.print(flow);lcd.print(" l/h ");
      lcd.setCursor(0,1);lcd.print("TempIN:");lcd.print(in_temp);lcd.print("*C ");
      lcd.setCursor(0,2);lcd.print("TempOUT:");lcd.print(out_temp);lcd.print("*C ");
      lcd.setCursor(0,3);lcd.print("TempTANK:");lcd.print(tank_temp);lcd.print("*C ");
    }
}

void flow_interrupt(){
  flow_pulse++;
}
/* ( THE END ) */

