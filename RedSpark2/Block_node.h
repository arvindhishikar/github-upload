
#include <stdio.h>
#include <stdlib.h>

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

unsigned char present_dir,plant_direction[7],Red_plant_destination[7];

int MakeDirectionNorth(int d)
{
	if (d==1)
	{
		stop();
	}
	else if (d==2)
	{
		left_degrees(90);
		stop();
		_delay_ms(100);
	}
	else if (d==3)
	{
		left_degrees(180);
		stop();
		_delay_ms(100);
	}
	else if (d==4)
	{
		right_degrees(90);
		stop();
		_delay_ms(100);
	}
	d=1;
	return d;
	
}

int MakeDirectionEast(int d)
{
	if (d==1)
	{
		right_degrees(90);
		stop();
		_delay_ms(100);
	}
	else if (d==2)
	{
		stop();
	}
	else if (d==3)
	{
		left_degrees(90);
		stop();
		_delay_ms(100);
	}
	else if (d==4)
	{
		left_degrees(180);
		stop();
		_delay_ms(100);
	}
	d=2;
	return d;
}
int MakeDirectionSouth(int d)
{
	if (d==1)
	{
		left_degrees(180);
		stop();
		_delay_ms(100);
	}
	else if (d==2)
	{
		right_degrees(90);
		stop();
		_delay_ms(100);
	}
	else if (d==3)
	{
		stop();
	}
	else if (d==4)
	{
		left_degrees(90);
		stop();
		_delay_ms(100);
	}
	d=3;
	return d;
}

int find_plant_destination(unsigned int previous_plant,unsigned int next_plant)
{
	unsigned int destNode;
	int D;
	
	if ((next_plant<previous_plant)||(next_plant==previous_plant))
	{
		present_dir=1	;
	}
	else
	present_dir=3;
	
	if (present_dir==1)
	{
		if ((next_plant>=1)&&(next_plant<=6))
		{
			destNode=next_plant+7;
		}
		else if ((next_plant>=7)&&(next_plant<=12))
		{
			destNode=next_plant+8;
		}
		else if ((next_plant>=13)&&(next_plant<=18))
		{
			destNode=next_plant+9;
		}
		else if ((next_plant>=19)&&(next_plant<=24))
		{
			destNode=next_plant+10;
		}
		else if ((next_plant>=25)&&(next_plant<=30))
		{
			destNode=next_plant+11;
		}
		else
		{
			destNode=next_plant+12;
		}
		direction = &D;
	}
	
	else if (present_dir==3)
	{
		if ((next_plant>=1)&&(next_plant<=6))
		{
			destNode=next_plant-1;
		}
		else if ((next_plant>=7)&&(next_plant<=12))
		{
			destNode=next_plant;
		}
		else if ((next_plant>=13)&&(next_plant<=18))
		{
			destNode=next_plant+1;
		}
		else if ((next_plant>=19)&&(next_plant<=24))
		{
			destNode=next_plant+2;
		}
		else if ((next_plant>=25)&&(next_plant<=30))
		{
			destNode=next_plant+3;
		}
		else
		{
			destNode=next_plant+4;
		}
		
	}
	return destNode;
	
}

void Find_destNode_of_Red_plants()
{
	int i,present_pos,count1=0;
	for (i=0;i<7;i++)
	{
		Red_plant_destination[i]=-1;
		plant_direction[i]=-1;
	}
	for (i=0;Red_plants[i]!=-1;i++)
	{
		if (count1==0)
		{
			present_pos=3;
		}
		else
		present_pos=Red_plants[i-1];
		
		count1++;
		
		Red_plant_destination[i] = find_plant_destination(present_pos,Red_plants[i]);
		plant_direction[i]=present_dir;
		_delay_ms(1000);
	}
}

int ReachDestNode(int d)
{
	int i=0;
	for (int i=0;i<20;i++)
	{
		arr1[i]=87;
	}	l=0;	
	while (complete!=50)
		while(!(UCSRA) & (1<<RXC));		
	
	for (i=0;arr1[i]!=87;i++)
	{  
			d=navigate(arr1[i],arr1[i+1],d);
	}
		
	if (flag2==1)  //spark2 parth max
	{
		flag2=0;
		while (complete!=93)
		{
			uart_tx(93);
		}
		
		while (complete!=90);
		
		while(complete!=91);
		_delay_us(1);
		uart_tx(101);
	}
	
	if (flag3==2)  //firebird path max
	{
		flag3=0;
		while(complete!=93);
		_delay_ms(250);
		uart_tx(91);
		while (complete!=101);
	}
	
	if(flag4==3)  //spark1 path max
	{
		flag4=0;
		while(complete!=90);
		
		_delay_ms(250);
		uart_tx(91);
		while (complete!=101);
	}
	
	_delay_ms(1000);
	return d;
}