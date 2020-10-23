// A C program for Dijkstra's single source shortest
// path algorithm. The program is for adjacency matrix
// representation of the graph.

#include <stdio.h>
#include <limits.h>

#include <math.h>

#define F_CPU 7372800
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
int *q;

// Number of vertices in the graph
#define V 49

// A utility function to find the vertex with minimum distance
// value, from the set of vertices not yet included in shortest
// path tree



int findPath( int graph[V][V],int start, int finish,int obs)
{
	int queue[49],temp_ans[49],ans[49],d,a;
	int front=-1,i,u,v,x,k;
	int distance[49];
	int color[49];
	int rear=-1;
	int parent[49];
	/*-----------------------------------------------------------------------------------------------------------PATH FINDING STARTS HERE*/
	
	for(k=0;k<=48;k++)
	{ 
		parent[k]=-1;
		distance[k]=0;;
		color[k]=1;
		ans[k]=50;
        lcd_print(1,13,k,2);
	}
	
	color[start]=0;
	distance[start]=0;
	queue[++rear]=start;
	front++;
	while(front!=rear+1)    /**terminate when queue is empty**/
	{
		u=queue[front];
		lcd_print(2,4,front,2);
		
		_delay_ms(100);
        if(u==obs)
		{
		  front++;
		  color[u]=-1;
		  continue;
		}		
		  
		for(i=0;i<49;i++)
		{
			//lcd_print(2,4,front,2);
			lcd_print(2,10,i,2);
			_delay_ms(100);
			v=graph[u][i];
			if(v==1)
			{
				if(color[i]==1)
				{
					color[i]=0;
					distance[i]=distance[u]+1;
					parent[i]=u;
					queue[++rear]=i;
					lcd_print(2,7,rear,2);

				}
			}
		}
		
		front++;
		color[u]=-1;
	}
 
	x=0;
	i=finish;
	while(parent[i]!=-1)         /**found path is stored here**/
	{ 
		temp_ans[x++]=i; 
		i=parent[i];
	}
	 
	temp_ans[x]=start;
	i=0;
	while(x!=0)                 /**path reverse final**/
	ans[i++]=temp_ans[x--];
 
	ans[i]=temp_ans[0];
 

if(ans[0]==45)
{
	d=1;
	forward_mm(100);
}

for(a=0;ans[a]!=(-1);a++)  
{
	//lcd_print(2,6,ans[a],2);          /* display present node location*/
	d= navigate(ans[a],ans[a+1],d);  /*send present and next node to navigate function,that function returns current direction of bot*/
}

  q=&d;   /*save present direction of bot in q*/
	/*---------------------------path finding ends here-------------------------------------------------------------------------
	
 return ans[a-1];   /* return destination */

}*/


