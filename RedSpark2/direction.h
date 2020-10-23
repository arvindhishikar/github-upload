#include <stdio.h>
#include <stdlib.h>

#define F_CPU 7372800
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning
volatile unsigned int flag1=0,complete=0,receive_data=0,l=0,m=0,flag2;
char p=0;
int *direction;
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
volatile int arr1[20],flag3,flag4;
unsigned int Red_plants[7];
//int arr2[20],p=0;

/*//Function to configure LCD port
void lcd_buzzer_port_config (void)
{
	DDRC = DDRC | 0xFF;    //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80;  // all the LCD pins are set to logic 0 except PORTC 7
}*/
void color_sensor_pin_config(void)
{
	DDRD  = DDRD | 0xBF;  //set PD6 as input for color sensor output 10111111
	PORTD = PORTD | 0x40; //Enable internal pull-up for PORTD 6 pin
	DDRC = DDRC | 0xFF;   //sets all bits of PORTC as output pins
	PORTC = PORTC | 0x00; //logic low on all 8 pins
}

void color_sensor_pin_interrupt_init(void) //Interrupt 0 enable
{
	cli(); //Clears the global interrupt
	TCCR1A = TCCR1A | 0x00; //clear TCCR1A register
	TCCR1B = TCCR1B | 0xC1;   // TIMER CAPTURE is set to trigger with falling edge
	TCNT1 = TCNT1 | 0x0000 ; //clear TCNT1 register
	TIFR = TIFR & 0x00;   //clear interrupt flag
	TIMSK = TIMSK | 0X20; // Enable Interrupt TIMER CAPTURE for color sensor
	sei(); // Enables the global interrupt
}
//ADC pin configuration
void adc_pin_config (void)
{
	DDRA = 0x00;   //set PORTF direction as input
	PORTA = 0x00;  //set PORTF pins floating
}

void motion_pin_config (void)
{
	DDRB = DDRB | 0x0F;   //set direction of the PORTB3 to PORTB0 pins as output
	PORTB = PORTB & 0xF0; // set initial value of the PORTB3 to PORTB0 pins to logic 0
	DDRD = DDRD | 0x30;   //Setting PD4 and PD5 pins as output for PWM generation
	PORTD = PORTD | 0x30; //PD4 and PD5 pins are for velocity control using PWM
}


//Timer1 is configured for constant frequency and variable duty cyclr
//TIMER1 initialize - prescale:64
// WGM: 5) PWM 8bit fast, TOP=0x00FF
// desired value: 450Hz
// actual value: 450.000Hz (0.0%)
void timer1_init(void)
{
	TCCR1B = 0x00; //stop
	TCNT1H = 0xFF; //higher byte constant frequency value of PWM cycle
	TCNT1L = 0x01; //lower byte constant frequency value of PWM cycle
	OCR1AH = 0x00;
	OCR1AL = 0xFF;
	OCR1BH = 0x00;
	OCR1BL = 0xFF;
	// ICR1H  = 0x00;
	// ICR1L  = 0xFF;
	TCCR1A = 0xA1;
	TCCR1B = 0x0D; //start Timer
}

//Function to Initialize ADC
void adc_init()
{
	ADCSRA = 0x00;
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//UART0 initialisation
// desired baud rate: 9600
// actual: baud rate:9600 (0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
	UCSRB = 0x00; //disable while setting baud rate
	UCSRA = 0x00;
	UCSRC = 0x86;
	UBRRL = 0x2F; //set baud rate lo  //67 is for 16MHz 9600 baudrate
	UBRRH = 0x00; //set baud rate hi
	UCSRB = 0x98;
}



//This Function accepts the Channel Number and returns the corresponding Analog Value
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;	//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10;      //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	return a;
}


//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR1AH = 0x00;
	OCR1AL = left_motor;     // duty cycle 'ON' period of PWM out for Left motor
	OCR1BH = 0x00;
	OCR1BL = right_motor;    // duty cycle 'ON' period of PWM out for Right motor
}

//Function to configure INT1 (PORTD 3) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
	DDRD  = DDRD & 0xF7;  //Set the direction of the PORTD 3 pin as input
	PORTD = PORTD | 0x08; //Enable internal pull-up for PORTD 3 pin
}

//Function to configure INT0 (PORTD 2) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRD  = DDRD & 0xFB;  //Set the direction of the PORTD 2 pin as input
	PORTD = PORTD | 0x04; //Enable internal pull-up for PORTD 2 pin
}

//Function to initialize ports
void port_init()
{
	//lcd_buzzer_port_config();
	adc_pin_config();
	color_sensor_pin_config();
	motion_pin_config(); //robot motion pins config
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
}

void left_position_encoder_interrupt_init (void) //Interrupt 1 enable
{
	cli(); //Clears the global interrupt
	MCUCR = MCUCR | 0x08; // INT1 is set to trigger with falling edge
	GICR = GICR | 0x80;   // Enable Interrupt INT1 for left position encoder
	sei(); // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 0 enable
{
	cli(); //Clears the global interrupt
	MCUCR = MCUCR | 0x02; // INT0 is set to trigger with falling edge
	GICR = GICR | 0x40;   // Enable Interrupt INT5 for right position encoder
	sei(); // Enables the global interrupt
}

//ISR for right position encoder
ISR(INT0_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}

//ISR for left position encoder
ISR(INT1_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}

ISR(USART_RXC_vect)
{
receive_data=UDR;
//UDR=receive_data;
complete=receive_data;


if (flag1==99)
{
	flag2=1;
}
if (flag1==105)
{
	flag3=2;
	//lcd_print(1,5,flag2,1);
}
if (flag1==106)
{
	flag4=3;
}
if (flag1==53)
{
	arr1[l]=receive_data;
	l++;
}
if (flag1==86)
{
	Red_plants[p]=receive_data;
	p++;
}

flag1=receive_data;

//complete=receive_data;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortBRestore = 0;

	Direction &= 0x0F; 			// removing upper nibbel as it is not needed
	PortBRestore = PORTB; 			// reading the PORTB's original status
	PortBRestore &= 0xF0; 			// setting lower direction nibbel to 0
	PortBRestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTB status
	PORTB = PortBRestore; 			// setting the command to the port
}

void forward (void) //both wheels forward
{
	motion_set(0x06);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}


void stop (void)
{
	motion_set(0x00);
}

//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 12.85; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	
	while (1)
	{
		
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		{
			break;
		}
		//lcd_print(2,5,ShaftCountRight,3);
	}
	stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 12.92; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	while(1)
	{
		//Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		//Center_white_line = ADC_Conversion(4);	//Getting data of Center WL Sensor
		//Right_white_line = ADC_Conversion(5);	//Getting data of Right WL Sensor
		
		flag=0;
		
		// print_sensor(1,1,3);		//Prints value of White Line Sensor Left
		//print_sensor(1,5,4);		//Prints value of White Line Sensor Center
		//print_sensor(1,9,5);		//Prints value of White Line Sensor Right
		
		if( ADC_Conversion(4)>0x10)
		{
			flag=1;
			forward();
			velocity(255,255);
		}

		else if(( ADC_Conversion(3)<0x10) && ( ( ADC_Conversion(5)>0x09) || ( ADC_Conversion(4)>0x09) ) && (flag==0))
		{
			flag=1;
			forward();
			velocity(200,120);
		}

		else if(( ADC_Conversion(5)<0x10) && ( ( ADC_Conversion(3)>0x09) || ( ADC_Conversion(4)>0x09) ) && (flag==0))
		{
			flag=1;
			forward();
			velocity(120,200);
		}
		
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
		//lcd_print(2,1,ShaftCountRight,3);
	}
	stop(); //Stop robot
}


int forward_mm(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	left(); //Turn left
	angle_rotate(Degrees);
}

void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right(); //Turn right
	angle_rotate(Degrees);
}


//Function to initialize all the devices
void init_devices()
{
	cli(); //Clears the global interrupt
	port_init();  //Initializes all the port
	adc_init();
	uart0_init();
	timer1_init();
	color_sensor_pin_interrupt_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   // Enables the global interrupt
}
void BlackLineFollowerForNavigation()
{
	while(1)
	{
		flag=0;
	   if( ADC_Conversion(4)>12)
			{
				flag=1;
				forward();
				velocity(255,255);
				//velocity(255,118);
			}
			else if(( ADC_Conversion(3)<12) && ( ( ADC_Conversion(5)>9) || ( ADC_Conversion(4)>9) ) && (flag==0))
			{
				flag=1;
				forward();
				velocity(200,120);
			}
			
			else if(( ADC_Conversion(5)<12) && ( ( ADC_Conversion(3)>9) || ( ADC_Conversion(4)>9) ) && (flag==0))
			{
				flag=1;
				forward();
				velocity(120,200);
			}
			
			/*else if(( ADC_Conversion(5)<0) && ( ( ADC_Conversion(3)<threshould) && ( ADC_Conversion(4)<threshould) ) && (flag==0))
			{
				flag=1;
				forward();
				velocity(0,0);
			}*/
			if ( ( ( ADC_Conversion(3)>80) && ( ADC_Conversion(4)>80) )||(( ADC_Conversion(5)>80) &&(ADC_Conversion(4)>80)))
			{
				buzzer_on();
				_delay_ms(100);
				buzzer_off();
				//forward_mm(40);
				forward();
				_delay_ms(100);
				//stop();
				//_delay_ms(10000);
				break;
			}
	}	
}

int navigate(int presentloc,unsigned int Nextloc,int Direction)   //Arena navigation for robot
{	
	/* DIRECTION CONSIDERATIONS
	
	    NORTH=1
		EAST=2
		SOUTH=3
		WEST=4
		
	*/	
	
 if(Direction==1)                                               
 {
     if(Nextloc==(presentloc+1))
     {
      right_degrees(90);
	  Direction=2;
	  stop();
	  _delay_ms(500);
      lcd_print(1,14,2,3);
	  BlackLineFollowerForNavigation();
      //forward_mm(320);
	  stop();
     }
	 
     else if(Nextloc==(presentloc-1))
     {
      left_degrees(90);
	  Direction=4;
	  stop();
	  _delay_ms(500);
      lcd_print(1, 14,4,3);
	  BlackLineFollowerForNavigation();
      //forward_mm(320);
	  stop();
     }
	 
     else if(Nextloc==(presentloc-7))
     {
	  forward();
	  Direction=1;
      lcd_print(1, 14,1,3);
      BlackLineFollowerForNavigation();
	  stop();
     }
	 
     else if(Nextloc==(presentloc+7))
     {
      right_degrees(180);
	  Direction=3;
	  stop();
	  _delay_ms(500);
      lcd_print(1, 14,3,3);
      BlackLineFollowerForNavigation();
	  stop();
     }
 }
 
 else if(Direction==2)
 {
	 
     if(Nextloc==(presentloc+7))
     {
      right_degrees(90);
	  Direction=3;
	  stop();
	  _delay_ms(500);
      lcd_print(1, 14,3,3);
      BlackLineFollowerForNavigation();
	  stop();
     }
	 
     else if(Nextloc==(presentloc-7))
     {
      left_degrees(90);
	  Direction=1;
	  stop();
	  _delay_ms(500);
      lcd_print(1,14,1,3);
      BlackLineFollowerForNavigation();
	  stop();
     }
	 
     else if(Nextloc==(presentloc+1))
     {
      forward();
	  Direction=2;
      lcd_print(1,14,2,3);
      BlackLineFollowerForNavigation();
	  stop();
     }
	 
     else if(Nextloc==(presentloc-1))
     {
      right_degrees(180);
	  Direction=4;
	  stop();
	  _delay_ms(500);
      lcd_print(1,14,4,3);
      BlackLineFollowerForNavigation();
	  stop();
     }
 }
 
  else if(Direction==3)
 {
     if(Nextloc==(presentloc-1))
     {
      right_degrees(90);
	  Direction=4;
	  stop();
      _delay_ms(500);
      lcd_print(1,14,4,3);
      BlackLineFollowerForNavigation();
     }
	 
     else if(Nextloc==(presentloc+1))
     {
       left_degrees(90);
	   Direction=2;
	   stop();
	   _delay_ms(500);
      lcd_print(1,14,2,3);
      BlackLineFollowerForNavigation();
	  stop();
     }
	 
     else if(Nextloc==(presentloc+7))
     {
	  forward();
	  Direction=3;
      lcd_print(1,14,3,3);
      BlackLineFollowerForNavigation();
	  stop();
     }
	 
     else if(Nextloc==(presentloc-7))
     {
      left_degrees(180);
	  Direction=1;
	  stop();
      _delay_ms(500);
      lcd_print(1,14,1,3);
      BlackLineFollowerForNavigation();
	  stop();
     }
 }
 
 else if(Direction==4)
 {
     if(Nextloc==(presentloc+7))
     {
      left_degrees(90);
	  Direction=3;
	  stop();
	  _delay_ms(500);
      lcd_print(1,14,3,3);
      BlackLineFollowerForNavigation();
	  stop();
     }
	 
     else if(Nextloc==(presentloc-7))
     {
      right_degrees(90);
	  Direction=1;
	  stop();
	  _delay_ms(500);
      lcd_print(1,14,1,3);
      BlackLineFollowerForNavigation();
	  stop();
     }
	 
     else if(Nextloc==(presentloc-1))
     {
	  forward();
	  Direction=4;
      lcd_print(1,14,4,3);
      BlackLineFollowerForNavigation();
	  stop();
     }
	 
     else if(Nextloc==(presentloc+1))
     {
       left_degrees(180);
	   Direction=2;
	   stop();
      _delay_ms(500);
      lcd_print(1,14,2,3);
      BlackLineFollowerForNavigation();
	  stop();
     }

 }
 direction=&Direction;
 return Direction;

}
//code ends here



/*int navigate(int presentloc,unsigned int Nextloc,int D)   //Arena navigation for robot
{	
	/* DIRECTION CONSIDERATIONS
	
	    NORTH=1
		EAST=2
		SOUTH=3
		WEST=4
		
		
	
 if(D==1)                                               
 {
     if(Nextloc==(presentloc+1))
     {
      right_degrees(90);
	  D=2;
	  stop();
	  _delay_ms(500);
      lcd_print(1,14,2,3);
      forward_mm(320);
	  stop();
     }
	 
     else if(Nextloc==(presentloc-1))
     {
      left_degrees(90);
	  D=4;
	  stop();
	  _delay_ms(500);
      lcd_print(1, 14,4,3);
      forward_mm(320);
	  stop();
     }
	 
     else if(Nextloc==(presentloc-7))
     {
	  forward();
	  D=1;
      lcd_print(1, 14,1,3);
      forward_mm(320);
	  stop();
     }
	 
     else if(Nextloc==(presentloc+7))
     {
      right_degrees(180);
	  D=3;
	  stop();
	  _delay_ms(500);
      lcd_print(1, 14,3,3);
      forward_mm(320);
	  stop();
     }
 }
 
 else if(D==2)
 {
	 
     if(Nextloc==(presentloc+7))
     {
      right_degrees(90);
	  D=3;
	  stop();
	  _delay_ms(500);
      lcd_print(1, 14,3,3);
      forward_mm(320);
	  stop();
     }
	 
     else if(Nextloc==(presentloc-7))
     {
      left_degrees(90);
	  D=1;
	  stop();
	  _delay_ms(500);
      lcd_print(1,14,1,3);
      forward_mm(320);
	  stop();
     }
	 
     else if(Nextloc==(presentloc+1))
     {
      forward();
	  D=2;
      lcd_print(1,14,2,3);
      forward_mm(320);
	  stop();
     }
	 
     else if(Nextloc==(presentloc-1))
     {
      right_degrees(180);
	  D=4;
	  stop();
	  _delay_ms(500);
      lcd_print(1,14,4,3);
      forward_mm(320);
	  stop();
     }
 }
 
  else if(D==3)
 {
     if(Nextloc==(presentloc-1))
     {
      right_degrees(90);
	  D=4;
	  stop();
      _delay_ms(500);
      lcd_print(1,14,4,3);
      forward_mm(320);
     }
	 
     else if(Nextloc==(presentloc+1))
     {
       left_degrees(90);
	   D=2;
	   stop();
	   _delay_ms(500);
      lcd_print(1,14,2,3);
      forward_mm(320);
	  stop();
     }
	 
     else if(Nextloc==(presentloc+7))
     {
	  forward();
	  D=3;
      lcd_print(1,14,3,3);
      forward_mm(320);
	  stop();
     }
	 
     else if(Nextloc==(presentloc-7))
     {
      left_degrees(180);
	  D=1;
	  stop();
      _delay_ms(500);
      lcd_print(1,14,1,3);
      forward_mm(320);
	  stop();
     }
 }
 
 else if(D==4)
 {
     if(Nextloc==(presentloc+7))
     {
      left_degrees(90);
	  D=3;
	  stop();
	  _delay_ms(500);
      lcd_print(1,14,3,3);
      forward_mm(320);
	  stop();
     }
	 
     else if(Nextloc==(presentloc-7))
     {
      right_degrees(90);
	  D=1;
	  stop();
	  _delay_ms(500);
      lcd_print(1,14,1,3);
      forward_mm(320);
	  stop();
     }
	 
     else if(Nextloc==(presentloc-1))
     {
	  forward();
	  D=4;
      lcd_print(1,14,4,3);
      forward_mm(320);
	  stop();
     }
	 
     else if(Nextloc==(presentloc+1))
     {
       left_degrees(180);
	   D=2;
	   stop();
      _delay_ms(500);
      lcd_print(1,14,2,3);
      forward_mm(320);
	  stop();
     }

 }
 direction=&D;
 return D;

}
//code ends here*/


