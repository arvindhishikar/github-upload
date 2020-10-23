


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

unsigned int Red_plants[7],Blue_plants[7],Green_plant[7];

volatile unsigned long int pulse = 0; //to keep the track of the number of pulses generated by the color sensor
volatile unsigned long int red;       // variable to store the pulse count when read_red function is called
volatile unsigned long int blue;      // variable to store the pulse count when read_blue function is called
volatile unsigned long int green;     // variable to store the pulse count when read_green function is called

//Filter Selection
void filter_red(void)    //Used to select red filter
{
	//Filter Select - red filter
	PORTC = PORTC & 0xB0; //set S2 low
	PORTC = PORTC & 0x70; //set S3 low
}

void filter_green(void)	//Used to select green filter
{
	//Filter Select - green filter
	PORTC = PORTC | 0x40; //set S2 High
	PORTC = PORTC | 0x80; //set S3 High
}

void filter_blue(void)	//Used to select blue filter
{
	//Filter Select - blue filter
	PORTC = PORTC & 0xB0; //set S2 low
	PORTC = PORTC | 0x80; //set S3 High
}

void filter_clear(void)	//select no filter
{
	//Filter Select - no filter
	PORTC = PORTC | 0x40; //set S2 High
	PORTC = PORTC & 0x70; //set S3 Low
}

//Color Sensing Scaling
void color_sensor_scaling()		//This function is used to select the scaled down version of the original frequency of the output generated by the color sensor, generally 20% scaling is preferable, though you can change the values as per your application by referring datasheet
{
	//Output Scaling 20% from datasheet
	//PORTD = PORTD & 0xEF;
	PORTC = PORTC | 0x10; //set S0 high
	//PORTD = PORTD & 0xDF; //set S1 low
	PORTC = PORTC | 0x20; //set S1 high
}

unsigned int red_read(void) // function to select red filter and display the count generated by the sensor on LCD. The count will be more if the color is red. The count will be very less if its blue or green.
{
	//Red
	filter_red(); //select red filter
	pulse=0;
	_delay_ms(100); //capture the pulses for 100 ms or 0.1 second
	red = pulse;  //store the count in variable called red
	return red;   //return red_pulses

}

unsigned int green_read(void) // function to select green filter and display the count generated by the sensor on LCD. The count will be more if the color is green. The count will be very less if its blue or red.
{
	//Green
	filter_green(); //select green filter
	pulse=0;
	_delay_ms(100); //capture the pulses for 100 ms or 0.1 second
	green = pulse;  //store the count in variable called green
	return green;   //return green_pulses
}

unsigned int blue_read(void) // function to select blue filter and display the count generated by the sensor on LCD. The count will be more if the color is blue. The count will be very less if its red or green.
{
	//Blue
	filter_blue(); //select blue filter
	pulse=0; //reset the count to 0
	_delay_ms(100); //capture the pulses for 100 ms or 0.1 second
	blue = pulse;  //store the count in variable called blue
	return blue;   //return blue_pulses
}

 void Black_line_following()
 {
	 flag=0;flag1=0;
			
			if( ADC_Conversion(4)>12)
			{
				flag=1;
				forward();
				velocity(255,255); //255,117
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
			
			if ( ( ( ADC_Conversion(3)>80) && ( ADC_Conversion(4)>80) )||(( ADC_Conversion(5)>80) &&(ADC_Conversion(4)>80)))
			{
				flag1=1;
				buzzer_on();
				_delay_ms(100);
				buzzer_off();
				forward();
				_delay_ms(1200);
				stop();
				_delay_ms(100);
				
			}
 }
 void plants_arr_init()
 {
	 int i;
	 for (i=0;i<7;i++)
	 {
		 Red_plants[i]=-1;
		 Blue_plants[i]=-1;
		 Green_plant[i]=-1;
	 }
 }

void Arena_scanning ()
{
	unsigned int i=0,j=0,k=0,red_pulses,green_pulses,blue_pulses,SP_V=0,count=0,plant_block_number=35;
	plants_arr_init();
	color_sensor_scaling();
	 while(1)
	 {	
		   
	  j=ADC_Conversion(7);
	  if(j>100)
	  {
		  if(SP_V == 1)
		  {
			  
			Black_line_following();
			
		  }
		  else
		  {
		  SP_V = 1 ;
		  stop();
		  buzzer_on();
		  _delay_ms(100);
		  buzzer_off();
		 red_pulses=red_read(); //display the pulse count when red filter is selected
		  _delay_ms(500);
		  green_pulses=green_read(); //display the pulse count when green filter is selected
		  _delay_ms(500);
		  blue_pulses=blue_read(); //display the pulse count when blue filter is selected
		  _delay_ms(500);
		  
		  if ((red_pulses>green_pulses) && (red_pulses>blue_pulses))
		  {
			  PORTC=PORTC | 0x01;
			  _delay_ms(5000);
			 Red_plants[p]=plant_block_number;
			 p++;
		  }
		  else if ((green_pulses>red_pulses) && (green_pulses>blue_pulses))
		  {
			  PORTC=PORTC | 0x02;
			  _delay_ms(5000);
			  Green_plant[j]=plant_block_number;
			  j++;
		  }
		  else
		  {
			  PORTC=PORTC | 0x04;
			  _delay_ms(5000);
			  Blue_plants[k]=plant_block_number;
			  k++;
		  }
		  PORTC=PORTC & 0x30;
		  }	
		  forward_mm(80);	  
	  }
	   
	   else
	   {
		   SP_V = 0;
		   Black_line_following();
	   }
	  if (flag1==1)
	  {
		  count++;
		  flag1=0;
		  if (count<6)
		  {
			  plant_block_number = plant_block_number - 6;
		  }
		  else if (count==6)
		  {
			  left_degrees(180);
			  stop();
			  _delay_ms(500);
			  plant_block_number = plant_block_number + 1;
		  }
		  else if(count>6)
		  {
			  plant_block_number = plant_block_number + 6;
		  }			  
		  
		  if (count==12)
		  {
			  stop();
			  break;
		  }	  	
	  }
	}	
	
	while(complete!=81)   //till BlueSpark scanning Done
	  while(!(UCSRA) & (1<<RXC));
	  
	uart_tx(82);
	
	while(complete!=84)
	 while(!(UCSRA) & (1<<RXC));
	  
   for (i=0;Blue_plants[i]!=-1;i++)
   {
	   uart_tx(85);
	   uart_tx(Blue_plants[i]);
   }
	for (i=0;Green_plant[i]!=-1;i++)
	{
		uart_tx(87);
		uart_tx(Green_plant[i]);
	}
	uart_tx(88);
}