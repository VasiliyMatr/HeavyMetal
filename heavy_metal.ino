// TX and RX pins are used for servo and beep -> don't occupie those in debug mode
#define DEBUG 0

/* ========== DISPLAY ========== */

#define F_CPU 16000000UL      /* Define CPU Frequency e.g. here 8MHz */
#include <avr/io.h>     /* Include AVR std. library file */
#include <util/delay.h>     /* Include Delay header file */

#define LCD_Dir  DDRD     /* Define LCD data port direction */
#define LCD_Port PORTD      /* Define LCD data port */
#define RS PD2        /* Define Register Select pin */
#define EN PD3        /* Define Enable signal pin */

#include <Servo.h>

Servo servo;

#define SERVO_PIN 1                                                                                                                                                                                      

long long servo_angle = 5;

#define BEEP_PIN 0

void LCD_Command( unsigned char cmnd )
{
  LCD_Port = (LCD_Port & 0x0F) | (cmnd & 0xF0); /* sending upper nibble */
  LCD_Port &= ~ (1<<RS);    /* RS=0, command reg. */
  LCD_Port |= (1<<EN);    /* Enable pulse */
  _delay_us(1);
  LCD_Port &= ~ (1<<EN);

  _delay_us(200);

  LCD_Port = (LCD_Port & 0x0F) | (cmnd << 4);  /* sending lower nibble */
  LCD_Port |= (1<<EN);
  _delay_us(1);
  LCD_Port &= ~ (1<<EN);
  _delay_ms(2);
}


void LCD_Char( unsigned char data )
{
  LCD_Port = (LCD_Port & 0x0F) | (data & 0xF0); /* sending upper nibble */
  LCD_Port |= (1<<RS);    /* RS=1, data reg. */
  LCD_Port|= (1<<EN);
  _delay_us(1);
  LCD_Port &= ~ (1<<EN);

  _delay_us(200);

  LCD_Port = (LCD_Port & 0x0F) | (data << 4); /* sending lower nibble */
  LCD_Port |= (1<<EN);
  _delay_us(1);
  LCD_Port &= ~ (1<<EN);
  _delay_ms(2);
}

void LCD_Init (void)      /* LCD Initialize function */
{
  LCD_Dir = 0xFF;     /* Make LCD port direction as o/p */
  _delay_ms(20);      /* LCD Power ON delay always >15ms */
  
  LCD_Command(0x02);    /* send for 4 bit initialization of LCD  */
  LCD_Command(0x28);              /* 2 line, 5*7 matrix in 4-bit mode */
  LCD_Command(0x0c);              /* Display on cursor off*/
  LCD_Command(0x06);              /* Increment cursor (shift cursor to right)*/
  LCD_Command(0x01);              /* Clear display screen*/
  _delay_ms(2);
}


void LCD_String (char *str)   /* Send string to LCD function */
{
  int i;
  for(i=0;str[i]!=0;i++)    /* Send each char of string till the NULL */
  {
    LCD_Char (str[i]);
  }
}

void LCD_String_xy (char row, char pos, char *str)  /* Send string to LCD with xy position */
{
  if (row == 0 && pos<16)
  LCD_Command((pos & 0x0F)|0x80); /* Command of first row and required position<16 */
  else if (row == 1 && pos<16)
  LCD_Command((pos & 0x0F)|0xC0); /* Command of first row and required position<16 */
  LCD_String(str);    /* Call LCD string function */
}

void LCD_Clear()
{
  LCD_Command (0x01);   /* Clear display */
  _delay_ms(2);
  LCD_Command (0x80);   /* Cursor at home position */
}

/* ========== SETUP FSM ========== */


#define SFSM_STATE_TIME 0
#define SFSM_STATE_READY 1
#define SFSM_STATE_MENU 2

#define SFSM_MENU_TIME 0
#define SFSM_MENU_READY 1
#define SFSM_MENU_ITEMS 2

int SFSM__State = SFSM_STATE_MENU;
int SFSM__MenuPos = SFSM_MENU_TIME;

int SFSM__Time = 2;

void SFSM_OnSwitchIncrease() {
  if(SFSM__State == SFSM_STATE_MENU) { 
    SFSM__MenuPos = (SFSM__MenuPos + 1) % SFSM_MENU_ITEMS;
  }

  if(SFSM__State == SFSM_STATE_TIME) {
    ++SFSM__Time;
  }
}

void SFSM_OnSwitchDecrease() {
  if(SFSM__State == SFSM_STATE_MENU) { 
    SFSM__MenuPos = max(0, SFSM__MenuPos - 1);
  }

  if(SFSM__State == SFSM_STATE_TIME) {
    --SFSM__Time;
  }
}

void SFSM_OnButtonPress() {
  if(SFSM__State != SFSM_STATE_MENU) {
    SFSM__State = SFSM_STATE_MENU;
  } else {
    SFSM__State = SFSM__MenuPos;
  }
}

void SFSM_Update() {
  LCD_Clear();
  
  if(SFSM__State == SFSM_STATE_MENU) {
    LCD_String("===> Setup Menu <===");

    switch(SFSM__MenuPos) {
    case SFSM_MENU_TIME:
      LCD_String("> Setup Time");
      break;
    case SFSM_MENU_READY:
      LCD_String("> Begin process");
      break;
    }
  }

  if(SFSM__State == SFSM_STATE_TIME) {
    LCD_String("===> Setup Time <===");

    LCD_String(("> " + String(SFSM__Time) + " min").c_str());
  }
  
}

/* ========== MAIN FSM ========== */

#define MFSM_STATE_SETUP 0
#define MFSM_STATE_RUNNING 1
#define MSFM_STATE_FINISHED 2

int MFSM__State = MFSM_STATE_SETUP;
unsigned long MFSM__StartTime = 0;
unsigned long MFSM__EndTime = 0;


void MFSM_OnSwitchIncrease() {
  // Serial.println(1);
  if(MFSM__State == MFSM_STATE_SETUP) {
    SFSM_OnSwitchIncrease();
  }
}

void MFSM_OnSwitchDecrease() {
  // Serial.println(2);
  
  if(MFSM__State == MFSM_STATE_SETUP) {
    SFSM_OnSwitchDecrease();
  }
}

void MFSM_OnButtonPress() {
  if(MFSM__State == MFSM_STATE_SETUP) {
    SFSM_OnButtonPress();
  }
}

void MFSM_Update() {
  if(MFSM__State == MFSM_STATE_SETUP) {
    SFSM_Update();
  }

  if(SFSM__State == SFSM_STATE_READY) {
    SFSM__State = SFSM_STATE_MENU;
    MFSM__State = MFSM_STATE_RUNNING;

    MFSM__StartTime = millis();
    MFSM__EndTime = MFSM__StartTime + (unsigned long)SFSM__Time * 60000;
  }

  if(MFSM__State == MFSM_STATE_RUNNING) {
    unsigned long t = millis();
    
    if(t > MFSM__EndTime) {
      MFSM__State = MSFM_STATE_FINISHED;
    }
    
    unsigned long dt = MFSM__EndTime - t;
    
    LCD_Clear();

    LCD_String("===>Running Info<===");
    LCD_String(("> Left: " + String(dt / 60000) + "m, " + String((dt % 60000) / 1000) + "s").c_str());

    if (!DEBUG) {
      servo.write(servo_angle);
      servo_angle = -servo_angle;
    }
  }

  if(MFSM__State == MSFM_STATE_FINISHED) {
    LCD_Clear();

    LCD_String("===>  Finished  <===");

    if (!DEBUG) {
      for(int i = 0; i != 50; i = i + 1) {
        digitalWrite(BEEP_PIN, HIGH);
        delay(1);
        digitalWrite(BEEP_PIN, LOW);
        delay(1);
      }
    }
  }
}

/* ========== SELECTOR ========= */

#define SEL_PIN_A 11
#define SEL_PIN_B 12
#define SEL_PIN_BUTTON 9

unsigned long Selector__LastState;
unsigned long Selector__LastButton;

void Selector_Init() {
  pinMode(SEL_PIN_A, INPUT);
  pinMode(SEL_PIN_B, INPUT);
  pinMode(SEL_PIN_BUTTON, INPUT);

  Selector__LastState = (digitalRead(SEL_PIN_A) << 1) | (digitalRead(SEL_PIN_B) << 0);
  Selector__LastButton = digitalRead(SEL_PIN_BUTTON);
}

void Selector_Update() {
  int state = (digitalRead(SEL_PIN_A) << 1) | (digitalRead(SEL_PIN_B) << 0);

  if(Selector__LastState != state) { 
    if((Selector__LastState==3 && state==1) /*|| (Selector__LastState==0 && state==2)*/) { 
      MFSM_OnSwitchIncrease();  
    } else if((Selector__LastState==2 && state==0) /*|| (Selector__LastState==1 && state==3)*/) { 
      MFSM_OnSwitchDecrease();
  } 

    Selector__LastState = state; 
  }

  int button = digitalRead(SEL_PIN_BUTTON);
  if(button == 0 && button != Selector__LastButton) {
    MFSM_OnButtonPress();
  }

  Selector__LastButton = button;
}

int last_render_time;

void setup() {
  delay(200);
  
  // put your setup code here, to run once:
  Selector_Init();
  LCD_Init();

  if (!DEBUG) {
    servo.attach(SERVO_PIN);
    pinMode(BEEP_PIN, OUTPUT);
  }

#if DEBUG
  Serial.begin(9600);
#endif

  last_render_time = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  Selector_Update();
  
  int t = millis();
  if(t - last_render_time >= 200) {
    MFSM_Update();
    last_render_time = t;
  }

  delay(1);
}
