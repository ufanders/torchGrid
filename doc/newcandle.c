//New Version of the Final Code for ECE693
//Philip Ching  
//02-02-2002: New Statemachine            
//02-05-2002: Changing statemachine   
//02-07-2002: Continued fixing dynamics 
//02-08-2002: Continued fixing dynamics  
//02-09-2002: Finishing up fixing dynamics, done.
//02-14-2002: Plugged in new 8535 into independent candle, observed new dynamics.

//------------------------------------------------------------------------------------------------------------
/* Usual includes... */
//------------------------------------------------------------------------------------------------------------
#include <90S8535.h>          
#include <stdio.h>  
#include <stdlib.h> 
#include <delay.h>  
#include <string.h> 
#include <math.h>      
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//Definitions for the flame dynamics
//------------------------------------------------------------------------------------------------------------
//Read Voltage on ANALOG.0
/* The operation modes(states) */ 
#define START				10			// Initial state for heating up thermistor
#define	REHEAT          	11     		// Need to wait fot thermistor to reheat
#define	NOBREEZE			12     		// No breeze is blowing
#define	LIGHTBREEZE 		13			// A light breeze is blowing
#define	STRONGBREEZE   		14         	// A strong breeze is blowing
#define BLOWNOUT			15	    	// Was the breeze strong enough to blow the candle out?

/* some parameters */ 
#define V_LIGHT				2 			// value or condition dividing NoBreeze and LightBreeze  (still air breezes)
#define V_STRONG			5			// value or condition dividing LightBreeze and StrongBreeze
#define V_BLOWOUT			8    		// value or condition dividing StrongBreeze and Blownout
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
                                                                                                              
 
 
//------------------------------------------------------------------------------------------------------------
/* interval timing stuff */ 
//------------------------------------------------------------------------------------------------------------
#define SCAN_INTERVAL	200 			// scan every 200 ms
#define WAIT			50				// How long to wait for themistor(heat) before we read ADC for Vzero.
										// Where the wait time = WAIT * SCAN_INTERVAL ms
#define eToMinusOne	0.3678794412    	// e^-1
#define TAU			2
#define blow	  	200
#define I_offset   	32 					// also add to get 255.	
#define I_scale    	200					// Used to multiply the random number and
#define PreScal0   	3     				// timer0 prescale by 64
#define PreScal1   	1     				// timer1 prescale 
#define PWMnorm    	0x80 	   			// pwm on, noninverted
#define PWM8bit    	1    				// 8-bit pwm mode  
#define t1 	  		64					// timeout values for each task  
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
/* function prototypes */
//------------------------------------------------------------------------------------------------------------
void initialize(void);          		// init everything
int performConversion(void);			// reads pinA.0 and performs ADC (actually, analog.0!!!)
void nobreeze(void);            		// Change parameters of LED when there's a NO breeze
void light(void);               		// Change parameters of LED when there's a light breeze
void strong(void);  					// Change parameters of LED when there's a strong breeze 
void task1(void);           			// random number generator and z calculator  
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
/* variables */          
//------------------------------------------------------------------------------------------------------------
// Timer stuff
unsigned char timer0_reload;    		// timer 0 reload time
unsigned char scan_time;  


//Thermistor Stuff
unsigned char converted, readyToConvert;   
int voltage, upper;
int oldvoltage;

//State-machine Stuff
unsigned char state;                	// Current state in statemachine
unsigned char prevState;		    	// Previous state        
unsigned char counter;					// Used in Start State, how long you wait intially for thermistor before reading ADC.

// State Variables   
int Vzero;								// "Zero" ADC value
int Vnew;				 				// New ADC voltage 
int deltaV;		       					// Change in Voltage,   
                  
//PWM Stuff
float e_to_minus_one;
float deltat;
float exponent;
float alpha;
float beta;                               

//Flame-Dynamic STuff
float Iscale;
float Ioffset;
float Blow;
float tau;

//For Task1
unsigned long int next;
unsigned int randomNum;

//For Timer0 interrupt
unsigned int readback;
unsigned int castedfloat;   
float finalRandomNum;
float putInOCR1A; 
float z_t; 								//z at time t
float z_t1;								//z at time t+1

//For Timer1                       
unsigned int  time1;					//task schedule-ing timeout counters
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------   



//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
/* The code starts here */
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
/* timer 0 overflow ISR -- setup the intervals for manual controller check */					
//------------------------------------------------------------------------------------------------------------
interrupt [TIM0_OVF] void timer0_overflow(void)
{        
	//reload to force 1 mSec overflow
  	TCNT0 = timer0_reload;
                      
	if(scan_time > 0) scan_time--;  
	if (time1>0)   	  --time1;		//Decrement the 
									//three times if they are not 
									//already zero
}
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
/* timer 1 overflow ISR */
//------------------------------------------------------------------------------------------------------------
interrupt [TIM1_OVF] void cmpA_overflow(void)
{       
	castedfloat = putInOCR1A;
	OCR1A = castedfloat;
	readback = OCR1A; 
}      
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------ 
/* ADC conversion ready ISR */
interrupt [ADC_INT] void adc_interrupt(void)
{
	converted = 1;
}
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
/* main controls everything */        
//------------------------------------------------------------------------------------------------------------
void main (void)    
{          
	initialize();	// initialize     
	
	while(1)     
	{    
		if (time1==0)
		{
			task1();       
		}
	    
	    if(scan_time == 0 && readyToConvert)
	    {     
	    
			scan_time = SCAN_INTERVAL;
	    
	    	if(state == START)
			{     
				printf("Start\r\n");
			 	if(counter == WAIT)
			 	{       
			 		// we waited (WAIT*200ms)sec's for the thermistor to fully heat up.
			 		Vzero = performConversion();
			 		printf("Vzero: %d\r\n",Vzero);
			 		state = NOBREEZE;
			 		prevState = NOBREEZE;
			 		nobreeze();			// load the nobreeze parameters
			 	}
			  	else        
		  		{                       
		  			counter++;			// increment the counter
				}	
			}
			
			else if(state == NOBREEZE)
			{     
				Vnew = performConversion();
				deltaV = abs(Vzero - Vnew);				
				printf("Current deltaV:  %d\r\n", deltaV); 
				if( deltaV <= V_LIGHT )
			  	{   
			  		state = NOBREEZE;  				// Still no breeze (there might be tiny air fluctuations though)
			  		if(prevState != NOBREEZE)
		  		 	{       
		  		 		printf("NoBreeze, ");
		  		      	nobreeze();
		  		  	}
		  		}
		   		else        
		  		{                       
		  			state = LIGHTBREEZE;
				}       
				prevState = NOBREEZE;			// Record that you've been in this state    
			}
			  
			else if(state == REHEAT)
			{                  
			  	Vnew = performConversion();              
	  			deltaV = abs(Vzero - Vnew);
	  			printf("REHEATING,deltaV:%d\r\n", deltaV);
	  		 	if( deltaV < V_LIGHT )
				{
		  		      state = NOBREEZE;  
		  		}
				//else the thermistor is not fully heated, so keep on heating...
		  	}
		             
		   else if(state == LIGHTBREEZE)
		   {
		       	
		       	if( (V_LIGHT <= deltaV) && (deltaV < V_STRONG) )
			  	{    
			  		if(prevState != LIGHTBREEZE)
			  		{   
			  			printf("LightBreeze, ");
			  			light();		// Load lightbreeze parameters	
			  		}	
			  		state = REHEAT; 
			  	}
		   		else        
		  		{                       
		  			state = STRONGBREEZE;
				}
				prevState = LIGHTBREEZE;
		  	}
		  	
		  	else if(state == STRONGBREEZE)
		  	{
		  				        
			  	if( (V_STRONG <= deltaV) && (deltaV < V_BLOWOUT) )
			  	{       
			  		if(prevState != STRONGBREEZE)
			  		{  
			  			printf("StrongBreeze, ");
			  			strong();		//load strongbreeze parameters
			  		}	                
			  		state = REHEAT;  
			  	}
		   		else        
		  		{                       
		  			state = BLOWNOUT;
				}
				prevState = STRONGBREEZE;
		  	}
          
     		else if(state == BLOWNOUT)
		  	{     
			  	if(V_BLOWOUT > deltaV)
			  	{       
			  		printf("Uh-oh, Problem!");  
			  	}
			  	printf("BLOWNOUT!!!!!!!!!!\r\n");
		   		DDRD = 0x00; 			// set PORTB to be all outputs
		   		PORTB = 0xff;			// and turn them off
		   		
		   		// and Shut off interrupts, candle is out
		   		#asm
		   		cli
		   		#endasm
       		}		  	
       		
		  	else
		  	{
				printf("Problem!\r\n");
		  	}	  
		  
	   } 
	    
	}// end while    		    
	
}// end main()
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
/*** conversion function ***/
//------------------------------------------------------------------------------------------------------------
int performConversion(void)
{
	ADMUX = 0x00;  			// select bit 0 to convert
	ADCSR.6 = 1;   			// start conversion   
 	
 	while(converted != 1);  // spin until conversion is done
	
	oldvoltage = voltage;	//save old value  
  	// read in value
	converted = 0;
	voltage = ADCL;
	upper = ADCH;
	upper = upper << 8;
	voltage = upper | voltage;
	return voltage;    
}    
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
/* Functions for loading parameters */
//------------------------------------------------------------------------------------------------------------
void nobreeze(void)
{
	//Change parameters of LED when there's a NO breeze
 	printf("NoBreezeParam\r\n");
   	#asm
	cli
	#endasm
 	Iscale = 50.00;
	Ioffset= 32.00;
	Blow   = 200.0;
	tau    = 20.00;
	alpha  = 0.99681;
	beta = 1 - alpha; 
   	#asm
	sei
	#endasm               	
}


void light(void)
{
	//Change parameters of LED when there's a light breeze    
	printf("LightBreezeParam\r\n");
	#asm
	cli
	#endasm
   	Iscale = 50.00;
	Ioffset= 64.00;
	Blow   = 200.0;
	tau    = 0.001;
	alpha  = 0.00166;
	beta = 1 - alpha; 
   	#asm
	sei
	#endasm   
}

void strong(void)
{
	//Change parameters of LED when there's a srong breeze  
	printf("StrongBreezeParam\r\n");
	#asm
	cli
	#endasm
   	Iscale = 200.00;
	Ioffset= 0.00;
	Blow   = 255.0;
	tau    = 0.00001;
	alpha  = 0.00166;
	beta = 1 - alpha; 
   	#asm
	sei
	#endasm   
}
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//Task 1
//------------------------------------------------------------------------------------------------------------
void task1(void) 
{   
  	time1 = t1;  				//reset the task timer  
  	
  	next = next * 1103515245 + 12345;
	randomNum = (unsigned int)(next/65536) % 32768;
	finalRandomNum = randomNum;
	finalRandomNum = finalRandomNum/32767 - 0.5;   
	
	z_t1 = (z_t * alpha) + (beta * finalRandomNum);
	z_t  = z_t1;   
	putInOCR1A = (z_t * I_scale) + I_offset + (blow * finalRandomNum);
  
}
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
/* initialize everything */  
//------------------------------------------------------------------------------------------------------------
void initialize(void) 
{             
	/* setup the serial UART */
	UCR = 0x10 + 0x08 ;
	UBRR = 25 ;        
 
	/* setup the timer 0 */     
	/* 62.5 x (64 x0.25) microSec = 1.0 mSec, so prescale 64, and 
	count 62 times. */ 
	timer0_reload = 256-62;		// value for 1 ms    
	TCNT0 = timer0_reload;		// preload timer 0 so that it  
								//interrupts after 1 ms  
	TCCR0 = PreScal0;			// prescalar to 64    
	TIMSK = 1;    
	
	//set up timer 1
	TIMSK  = 0b00000100 | 1;	//enable timer1 ovfl intr
 	TCCR1B = PreScal1;	  		//prescale timer1	
	TCCR1A = PWMnorm+PWM8bit; 	//8-bit pwm, output A normal         
    	
 	/*setup the ports*/
  	DDRD  = 0xff;             	// set to all ones so that PORT D = 
								// all outputs
	PORTD = 0;                
	DDRB  = 0xff;
	PORTB = 0xff;               // Lights off
    
	//PORT A initializations
	//initialize portA to be input
	DDRA = 0xfe;
	PORTA = 0xfe;  
             
	/* ADC stuff */
	// set ADC Enable (bit 7), in single conversion mode, enable 
    // interrupt flag
	// 1000 1110  // prescale 64  
	ADCSR = 0x8e;          
  
	readyToConvert = 1;
	converted = 0;
    
	/* interval timing stuff */
	scan_time = SCAN_INTERVAL; 
	
	//init the task timer
  	time1 = t1;
	              
	//control operation mode stuff
	state = START;
	prevState = START;
	
	//for adc conversion
	Vnew = 0; 
    oldvoltage = 0;
	voltage = 0;  
	counter = 0; 
    deltaV = 0; 

	//for flame dynamics		
	e_to_minus_one = eToMinusOne;
	Iscale  = I_scale;
	Ioffset = I_offset;
	Blow    = blow;
	tau = TAU;
    deltat = t1/1000.00;
	exponent = deltat/tau; 

	//start the candle with nobreeze params   
    Iscale = 50.00;
	Ioffset= 32.00;
	Blow   = 200.0;
	tau    = 20.00;
	alpha  = 0.99681;
	beta = 1 - alpha; 
    //end dynamics
    	        
 	next = 1;  
	finalRandomNum = 0;
 	putInOCR1A =0;
    z_t  = 0;
	z_t1 = 0;
    
	printf("initialized.\r\n");
 	#asm 
	sei 
	#endasm
}   
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
