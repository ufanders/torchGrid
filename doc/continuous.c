//New Version of the Final Code for ECE693
//Philip Ching  
//02-02-2002: New Statemachine            
//02-05-2002: Changing statemachine   
//02-07-2002: Continued fixing dynamics 
//02-08-2002: Continued fixing dynamics  
//02-09-2002: Finishing up fixing dynamics, done. 
//02-14-2002: Plugged into offboard candle
//02-15-2002: demo'd  

//Next Version
//02-18-2002: taking out statemachine, adding dynamic initial read, adding continuous controller.     
//02-19-2002: testing, finding parameter relationship, need to fix tau and alpha
//02-20-2002: continued editing code   
//02-21-2002: test code

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
/* interval timing stuff */ 
//------------------------------------------------------------------------------------------------------------
#define SCAN_INTERVAL		200				// scan every 200 ms
#define WAIT				50				// How long to wait for themistor(heat) before we read ADC for Vzero.
											// Where the wait time = WAIT * SCAN_INTERVAL ms
#define eToMinusOne			0.3678794412	// e^-1
#define TAU					2						
#define blow	  			200
#define I_offset   			32				// also add to get 255.	
#define I_scale    			200				// Used to multiply the random number and
#define PreScal0			3   			// timer0 prescale by 64
#define PreScal1			1   			// timer1 prescale 
#define PWMnorm				0x80   			// pwm on, noninverted
#define PWM8bit				1  				// 8-bit pwm mode  
#define t1					64				// timeout values for each task  
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
#define iScale_multiplier	4.0				// factor for multiplying Iscale     
#define iOffset_multiplier	2.0				// factor for multiplying Ioffset  
#define blow_multiplier		25.0			// for scaling Blow variables
#define alpha_multiplier	150.0			// factor for dividing multiplier
   
#define OUT					20				// deltaV considered enough to blow candle out
//------------------------------------------------------------------------------------------------------------
/* function prototypes */
//------------------------------------------------------------------------------------------------------------
void initialize(void);          			// init everything  
void read(void);							// Used to read the initial ADC Vzero
int performConversion(void);				// reads pinA.0 and performs ADC (actually, analog.0!!!)
void task1(void);           				// random number generator and z calculator  
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
/* variables */          
//------------------------------------------------------------------------------------------------------------
// Timer stuff
unsigned char timer0_reload;    			// timer 0 reload time
unsigned char scan_time;  


//Thermistor Stuff
unsigned char converted, readyToConvert;   
int voltage, upper;
int oldvoltage;


// Continuous Controller Variables   
int Vzero;								// "Zero" ADC value
int Vnew;				 				// New ADC voltage 
int deltaV;		       					// Change in Voltage
int history[5];							// ADC history 
char reading;							// Are we still reading the initial value? 
char closeEnough;						// for dynamic ADC Vzero determination 
char outstr[32];						// Outstring after float to char conversion
                  
//PWM Stuff
float e_to_minus_one;
float deltat;
float exponent;
float alpha;
float beta;                               

//Flame-Dynamics Stuff
float Iscale;
float Ioffset;
float Blow;
float tau;         

//Continuous Controller Stuff
float Iscale_default;
float Ioffset_default;
float Blow_default;
float alpha_default;

//For Task1
unsigned long int next;
unsigned int randomNum;

//For Timer0 interrupt
unsigned int readback;
unsigned int castedfloat;   
float finalRandomNum;
float putInOCR1A; 
float z_t; 						 		//z at time t
float z_t1;						 		//z at time t+1

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
	if (time1>0)   	  --time1;			//Decrement the 
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
			if(reading)	
			{
				read();
			}
			else
			{
				Vnew = performConversion();  
				deltaV = abs(Vnew - Vzero);
			    
			    if(deltaV > 3)
				{

					//Changing Iscale parameter
					//flame gets dimmer with strong breeze
					//Iscale = 25.00;
					Iscale = Iscale_default	- (deltaV * iScale_multiplier);  
					//Iscale = Iscale_default;
					
					//Changing Ioffset parameter
					//flame gets dimmer with strong breeze
					//Ioffset = Ioffset_default - (deltaV * iOffset_multiplier);              
    	Ioffset = Ioffset_default; 
                                                                   
					// Changing Blow parameter 
					//flame gets increasingly disturbed with strong breeze
					Blow = Blow_default + (deltaV * blow_multiplier);
			        
					// Changing alpha parameter
					//flame gets 'faster' with strong breeze		
					alpha  = alpha_default/(alpha_multiplier * deltaV);
					//alpha  = alpha_default;
					beta = 1 - alpha; 

				}
				else
	            {
                        
					Iscale = Iscale_default;
					Ioffset = Ioffset_default;
					Blow = Blow_default;
					alpha = alpha_default; 
					beta = 1 - alpha;
			
            	}
                
            	// Blowout Condition Check
            	if(deltaV >= OUT)
            	{
            		printf("BLOWNOUT!!!!!!!!!!\r\n");
		   			DDRD = 0x00; 			// set PORTB to be all outputs
		   			PORTB = 0xff;			// and turn them off
		   		
		   			// and Shut off interrupts, candle is out
		   			#asm
		   			cli
		   			#endasm
            
           		}
                       
                        
					// status printouts
					printf("deltaV: %d\r\n",deltaV);
					//ftoa(Blow,2,outstr); 		     				//convert the float to a string
					//printf("Blow = %s\r\n",outstr);      			//and print it            
	      	}
	     } 
	    
	}// end while    		    
	
}// end main()
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
                           


//------------------------------------------------------------------------------------------------------------
/*** Read function ***/
//------------------------------------------------------------------------------------------------------------
void read(void)     
{      
	closeEnough = 0;
	Vnew = performConversion(); 
	printf("Vnew: %d\r\n",Vnew); 
	
	// check for ADC bouncing
	if(Vnew == history[0])	closeEnough++;
	if(Vnew == history[1])	closeEnough++;
	if(Vnew == history[2])	closeEnough++;                
	if(Vnew == history[3]) 	closeEnough++;	  
	if(Vnew == history[4]) 	closeEnough++;	  
	
	// matches at least 2 of the past ADC recorded values
	if(closeEnough >= 3)
	{
		Vzero = Vnew;
		printf("Vzero: %d\r\n",Vzero);
		reading = 0;			// done reading ADC for Vzero
		printf("---reading complete---\r\n");
	}
	else
	{         
		// update three most recent ADC values
		history[0] = history[1];
		history[1] = history[2];
		history[2] = history[3];
		history[3] = history[4];
		history[4] = Vnew;    
	}
}
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
/*** conversion function ***/
//------------------------------------------------------------------------------------------------------------
int performConversion(void)
{
	ADMUX = 0x00;  				// select bit 0 to convert
	ADCSR.6 = 1;   				// start conversion   
 	
 	while(converted != 1);  	// spin until conversion is done
	
	oldvoltage = voltage;		// save old value  
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
	TCNT0 = timer0_reload;		// preload timer 0 so that it interrupts after 1 ms  
	TCCR0 = PreScal0;			// prescalar to 64    
	TIMSK = 1;    
	
	//set up timer 1
	TIMSK  = 0b00000100 | 1;	//enable timer1 ovfl intr
 	TCCR1B = PreScal1;	  		//prescale timer1	
	TCCR1A = PWMnorm+PWM8bit; 	//8-bit pwm, output A normal         
    	
 	/*setup the ports*/
  	DDRD  = 0xff;             	// set to all ones so that PORT D = all outputs
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
	
	//for adc conversion
	Vnew = 0; 
	oldvoltage = 0;
	voltage = 0;  
 	deltaV = 0; 

	//for flame dynamics		
	e_to_minus_one = eToMinusOne;
	Iscale  = I_scale;
	Ioffset = I_offset;
	Blow    = blow;
	tau 	= TAU;
 	deltat = t1/1000.00;
	exponent = deltat/tau; 

	//start the candle with nobreeze params   
 	Iscale = 50.00;
	Ioffset= 32.00;
	Blow   = 200.0;
	tau    = 20.00;
	alpha  = 0.99681;
	beta 	= 1 - alpha; 
 	//end dynamics
    
    Iscale = 25.00;	
	Iscale_default = Iscale;
	Ioffset_default = Ioffset;
	Blow_default = Blow;
	alpha_default = alpha;
    	        
 	next = 1;  
	finalRandomNum = 0;
 	putInOCR1A =0;
	z_t  = 0;
	z_t1 = 0; 
	
	//Continuous controller
	reading = 1;	// we want to read the initial ADC value for Vzero  
	history[0] = 123;
	history[1] = 456;
	history[2] = 789;
	history[3] = 123;
	history[4] = 456;
            
	printf("initialized\r\n");
 	#asm 
	sei 
	#endasm
}   
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
