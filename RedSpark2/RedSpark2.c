/*
 * Spark2_path.c
 *
 * Created: 16/02/2018 10:28:12 AM
 *  Author: Madhuri
 */ 

#define F_CPU 7372800
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

#include "buzzer.h"
#include "lcd.h"
#include "direction.h"
#include "Block_node.h"
#include "Arena_scanning.h"
#define TE (1<<5)

void uart_tx(int data)
{
	while(!(UCSRA & TE));
	UDR=data;
}

int main(void)
{
	init_devices();
	int i,j,Robot_direction=2;	
	Robot_direction=ReachDestNode(Robot_direction);
	Arena_scanning ();
	Find_destNode_of_Red_plants();
	 Red_plant_destination[p]=7;  //storing node near to Red home
	while (complete!=56)
		while(!(UCSRA) & (1<<RXC));
	
	for (i=0;Red_plant_destination[i]!=-1;i++)
	{
		uart_tx(55);
		_delay_ms(100);
		uart_tx(Red_plant_destination[i]);
		
	}
	_delay_ms(150);
	uart_tx(57);  //done with sending all nodes
	
	for (j=0;Red_plant_destination[j]!=-1;j++)
	{
		Robot_direction=ReachDestNode(Robot_direction);
	}
	Robot_direction = MakeDirectionEast(Robot_direction);
	forward_mm(70);
	stop();
}