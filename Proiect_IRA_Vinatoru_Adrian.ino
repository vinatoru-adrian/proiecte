#include <LiquidCrystal.h>

#define BUTTON_OK       6
#define BUTTON_CANCEL   7
#define BUTTON_PREV     8
#define BUTTON_NEXT     9

enum Buttons
{
  EV_OK, 
  EV_CANCEL,
  EV_NEXT,
  EV_PREV,
  EV_NONE,
  EV_MAX_NUM
};

enum Menus
{
  MENU_MAIN=0,
  MENU_T_SET,
  MENU_KP,
  MENU_KI,
  MENU_KD,
  MENU_T_INCALZIRE,
  MENU_T_MENTINERE,
  MENU_T_RACIRE,
  MENU_PID,
  MENU_MAX_NUM
};

LiquidCrystal lcd(12, 11, 5, 4, 10, 2);

double t_set = 28;
double kp = 60;
double ki = 0.4;
double kd = 0.096;
double timp_i = 180;
double timp_m = 50;
double timp_r = 500;

double temp_initiala = 0;
double setpoint = 0;
double pid_timp = 0;
int nr_sec = 0;

unsigned int numar_CAN;

Menus scroll_menu = MENU_MAIN;
Menus current_menu = MENU_MAIN;

void state_machine( enum Menus menu, enum Buttons button);
Buttons GetButtons(void);
void print_menu( enum Menus menu);

typedef void (state_machine_handler_t)(void);

void print_menu(enum Menus menu)
{
  lcd.clear();
  switch(menu)
  {
    case MENU_T_SET:
        lcd.setCursor(0,0);
        lcd.print("TEMP SET = ");
        lcd.print(t_set);
        break;
        
    case MENU_KP:
        lcd.setCursor(0,0);
        lcd.print("KP = ");
        lcd.print(kp);
        break;

   case MENU_KI:
        lcd.setCursor(0,0);
        lcd.print("KI = ");
        lcd.print(ki);
        break;

   case MENU_KD:
        lcd.setCursor(0,0);
        lcd.print("KD = ");
        lcd.print(kd);
        break;

   case MENU_T_INCALZIRE:
        lcd.setCursor(0,0);
        lcd.print("TIMP I = ");
        lcd.print(timp_i);
        break;
        
   case MENU_T_MENTINERE:
        lcd.setCursor(0,0);
        lcd.print("TIMP M = ");
        lcd.print(timp_m);
        break;
        
   case MENU_T_RACIRE:
        lcd.setCursor(0,0);
        lcd.print("TIMP R = ");
        lcd.print(timp_r);
        break;

   case MENU_PID:
        lcd.setCursor(0,0);
        lcd.print("TEMP PID= ");
        lcd.print(temperatura());
        break;
        
    case MENU_MAIN:
    default:
      lcd.setCursor(0,0);
      lcd.print("PS 2022");
      lcd.setCursor(0, 1);
      lcd.print("Vinatoru Adrian");
      break; 
  }
  if( (current_menu != MENU_PID) && (current_menu != MENU_MAIN) )
  {
    lcd.setCursor(0, 1);
    lcd.print("modifica");
  }
  if(current_menu == MENU_PID)
  {
    pid_setpoint();
  }
}

void enter_menu(void)
{
  current_menu = scroll_menu;
}

void go_home(void)
{
  scroll_menu = MENU_MAIN;
  current_menu = scroll_menu;
}

void go_next(void)
{
  scroll_menu = (Menus) ((int)scroll_menu + 1);
  scroll_menu = (Menus) ((int)scroll_menu % MENU_MAX_NUM);
}

void go_prev(void)
{
  scroll_menu = (Menus) ((int)scroll_menu - 1);
  scroll_menu = (Menus) ((int)scroll_menu % MENU_MAX_NUM);
}

void inc_t_set(void)
{
  t_set++;
}

void dec_t_set(void)
{
  t_set--;
}

void inc_kp(void)
{
  kp = kp + 0.1;
}

void dec_kp(void)
{
  kp = kp - 0.1;
}

void inc_ki(void)
{
  ki = ki + 0.1;
}

void dec_ki(void)
{
  ki = ki - 0.1;
}

void inc_kd(void)
{
  kd = kd + 0.1;
}

void dec_kd(void)
{
  kd = kd - 0.1;
}

void inc_timp_i(void)
{
  timp_i++;
}

void dec_timp_i(void)
{
  timp_i--;
}

void inc_timp_m(void)
{
  timp_m++;
}

void dec_timp_m(void)
{
  timp_m--;
}

void inc_timp_r(void)
{
  timp_r++;
}

void dec_timp_r(void)
{
  timp_r--;
}

void nimic(void)
{
  
}

state_machine_handler_t* sm[MENU_MAX_NUM][EV_MAX_NUM] =
{
  // evenimentele:
  // OK,        CANCEL,     NEXT,         PREV
  
  {enter_menu,  go_home,    go_next,      go_prev },                  // MENU_MAIN
  {go_home,     go_home,    inc_t_set,    dec_t_set },                // MENU_T_SET
  {go_home,     go_home,    inc_kp,       dec_kp },                   // MENU_KP
  {go_home,     go_home,    inc_ki,       dec_ki },                   // MENU_KI
  {go_home,     go_home,    inc_kd,       dec_kd },                   // MENU_KD
  {go_home,     go_home,    inc_timp_i,   dec_timp_i },               // MENU_T_INCALZIRE
  {go_home,     go_home,    inc_timp_m,   dec_timp_m },               // MENU_T_MENTINERE
  {go_home,     go_home,    inc_timp_r,   dec_timp_r },               // MENU_T_RACIRE
  {go_home,     go_home,    nimic,        nimic  },                   // MENU_PID
};

void state_machine(enum Menus menu, enum Buttons button)
{
  sm[menu][button]();
}

Buttons GetButtons(void)
{
  enum Buttons ret_val = EV_NONE;
  if (digitalRead(BUTTON_OK))
  {
    ret_val = EV_OK;
  }
  else if (digitalRead(BUTTON_CANCEL))
  {
    ret_val = EV_CANCEL;
  }
  else if (digitalRead(BUTTON_NEXT))
  {
    ret_val = EV_NEXT;
  }
  else if (digitalRead(BUTTON_PREV))
  {
    ret_val = EV_PREV;
  }
  
  return ret_val;
}

void adc_init() 
{ 
  ADCSRA |= ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
  ADMUX |= (1 << REFS0); 
  ADCSRA |= (1 << ADEN);
  ADCSRA |= (1 << ADSC);
}

uint16_t read_adc(uint8_t channel) 
{
  ADMUX &= 0xF0;
  ADMUX |= channel;
  ADCSRA |= (1<<ADSC); 
  while(ADCSRA & (1<<ADSC)); 
  return ADC;
}

double temperatura()
{
   double temp; 
   double valoare_volti; 
   numar_CAN = read_adc(4);  
   valoare_volti = (( (double)numar_CAN  *  5000) / 1023); 
   temp = (valoare_volti) / 10;
   return temp;
}

void timer_init()
{ // 0.5 sec
  
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1C = 0;

  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);   //PRESCALER 1024
  OCR1A = 7811; 
  TIMSK1 |= (1 << OCIE1A); 
}

void PWM_init()
{
  //timer 2 pentru pwm
  DDRD |= (1 << 3);                                   // PIN pentru PWM
  TCCR2A |= (1 << COM2B1);                            // NON INVERTING MODE
  TCCR2A |= (1 << WGM21) | (1 << WGM20);              // FAST PWM
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);  //PRESCALER 1024
  OCR2B = 255;
}

void PID()
{
  double eroare = 0;
  static double suma_erori = 0;
  double eroare_anterioara = 0;
  double derivativa = 0;
  double dt = 0.1;           // timp de esantionare
  double output = 0;

  eroare = setpoint - temperatura();
  suma_erori = suma_erori + eroare * dt;
  derivativa = (eroare - eroare_anterioara) / dt;
  output = (kp * eroare) + (ki * suma_erori) + (kd * derivativa);
  eroare_anterioara = eroare;

  if( output > 255)
  {
    OCR2B = 255;
  }
  else if( output < 0)
  {
    OCR2B = 0;
  }
  else 
  {
    OCR2B = (int)output;
  }
}
void pid_setpoint()
{ 
    int minute = 0; 
    int secunde = 0;
    int timp_ramas=0;
    
    if(scroll_menu == MENU_PID)
    {
        pid_timp++;
        
        lcd.setCursor(0, 1);
        lcd.print("TS:");
        lcd.print((int)t_set);
        
      
        if(pid_timp <= timp_i) 
        {
          lcd.setCursor(8, 1);
          lcd.print("Ti:");
          
          timp_ramas= timp_i- pid_timp;
          setpoint = temp_initiala + (t_set - temp_initiala)/(timp_i) * pid_timp;
          
          minute = timp_ramas / 60;
          secunde = timp_ramas % 60;
          lcd.print(minute);
          lcd.print(":");
          lcd.print(secunde);
        }
        else if(pid_timp <= (timp_i + timp_m))
        {
          lcd.setCursor(8, 1);
          lcd.print("Tm:");
          
          timp_ramas = (timp_i + timp_m) - pid_timp;
          setpoint = t_set;
          
          minute = timp_ramas / 60;
          secunde = timp_ramas % 60;
          lcd.print(minute);
          lcd.print(":");
          lcd.print(secunde);
        }
        else if(pid_timp <= timp_i + timp_m + timp_r)
        { 
          lcd.setCursor(8, 1);
          lcd.print("Tr:");
          
          timp_ramas = (timp_i + timp_m + timp_r) - pid_timp;
          setpoint = t_set - (t_set - temp_initiala) / (timp_r) * ((pid_timp - timp_i - timp_m));

          minute = timp_ramas / 60;
          secunde = timp_ramas % 60;
          lcd.print(minute);
          lcd.print(":");
          lcd.print(secunde);
        }
        else
        {
            setpoint = temp_initiala;
        }
    }
    else
    {
        pid_timp = 0;
    }
    
}
ISR ( TIMER1_COMPA_vect )
{
  nr_sec++;
  
  if (((nr_sec * 4) % 2) == 0)
  {
    nr_sec=0;
  }
  
  volatile Buttons event = GetButtons();
  if ( event != EV_NONE)
  {
    state_machine( current_menu, event); 
  }
  print_menu(scroll_menu);

  if(scroll_menu == MENU_PID)
  {
    PID();
    // Serial.println("temp= ");
    // Serial.println(temperatura());
  }
  else if (scroll_menu != MENU_PID)
  {
    temp_initiala = temperatura();
    OCR2B = 0;
  }
  if( nr_sec == 0)
  {
   //Serial.print("temp= ");
   Serial.println(temperatura());
 
   Serial.print("sp= ");
   Serial.println(setpoint);
   
   pid_setpoint();
  }
  
}

void setup() 
{
  Serial.begin(9600);
  lcd.begin(16, 2);

  sei();

  timer_init();
  PWM_init();
  adc_init();
  
  //butoane
  pinMode(6, INPUT);
  digitalWrite(6, LOW);
  
  pinMode(7, INPUT);
  digitalWrite(7, LOW);
  
  pinMode(8, INPUT);
  digitalWrite(8, LOW);
  
  pinMode(9, INPUT);
  digitalWrite(9, LOW);

}

void loop() 
{
  

}
