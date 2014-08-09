//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
//Final Code for M.eng
//ECE693
//12-7-2001: 	Combined 8515 Code with 8535  
//12-11-2001:	Updated Breeze Parameters, copied over the old state machine
//12-17-2001:	New Chip
//11-19-2001:	Tweek parameters, and finish
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
#include <90S8535.h>          
#include <stdio.h>  
#include <stdlib.h> 
#include <delay.h>  
#include <string.h> 
#include <math.h>  
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
//Read Voltage on ANALOG.0
/* The operation modes(states) */
#define	NOBREEZE		10        
#define BLOW			11  	
#define	LIGHT			12
#define	BLOWING	  		13
#define	STRONG    		14  
#define LIGHTHELPER		15  

/* interval timing stuff */
#define SCAN_INTERVAL	100 				// scan every 100 ms

/* some parameters */
#define MIN_V_CHANGE	2 					// minimum ADC change to be considered a breeze
#define DV_DT_STRONG	4 					// dV/dT considered to be a strong breeze
#define BLOWOUT			6

#define eToMinusOne		0.3678794412       	//e^-1
#define TAU		   		2
#define blow	  		200
#define I_offset   		32 					//also add to get 255.	
#define I_scale    		200					//Used to multiply the random number and
#define PreScal0   		3             		//timer0 prescale by 64
#define PreScal1   		1             		//timer1 prescale 
#define PWMnorm    		0x80          		//pwm on, noninverted
#define PWM8bit    		1             		//8-bit pwm mode  
#define t1 	  	  		64		    		//timeout values for each task

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
// State Variables
int Vnew;					  				// New ADC voltage
int Vold;		       						// Previous ADC voltage 
int deltaV;		       						// Change in Voltage                       

/* function prototypes */
void initialize(void);          			// init everything
int performConversion(void);    	       	// reads pinA.0 and performs ADC (actually, analog.0!!!)
void nobreeze(void);            	       	// Change parameters of LED when there's a NO breeze
void light(void);               			// Change parameters of LED when there's a light breeze
void strong(void);  						// Change parameters of LED when there's a strong breeze 
void task1(void);           				//random number generator and z calculator

/* variables */          
// timer stuff
unsigned char timer0_reload;    			// timer 0 reload time
unsigned char scan_time; 

//Thermistor Stuff
unsigned char converted, readyToConvert;   
int voltage, upper;
int oldvoltage;

//State-machine Stuff
unsigned char mode;                         // Current state in state-machine
unsigned char prevMode;						// Previous Mode    

//Variable section
float e_to_minus_one;
float deltat;
float exponent;
float alpha;
float beta;                               
//char outstr[32];							//Outstring after float to char conversion

float Iscale;
float Ioffset;
float Blow;
float tau;

//For Task1
unsigned long int next;
unsigned int randomNum;

//For Timer1 interrupt
unsigned int readback;
unsigned int castedfloat;   
float finalRandomNum;
float putInOCR1A; 
float z_t; 						   			//z at time t
float z_t1;									//z at time t+1

//For Timer0                       
unsigned int  time1;						//task schedule-ing timeout counters
//float curr_data;							//current code


//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
/* timer 0 overflow ISR -- setup the intervals for manual controller check */					
interrupt [TIM0_OVF] void timer0_overflow(void)
{        
	//reload to force 1 mSec overflow
  	TCNT0 = timer0_reload;
                      
	if(scan_time > 0) scan_time--;  
	if (time1>0)	--time1;		//Decrement the three times if they are not already zero
}

interrupt [TIM1_OVF] void cmpA_overflow(void)
{       
	castedfloat = putInOCR1A;
    OCR1A = castedfloat;
    readback = OCR1A; 
}      

        
/* ADC conversion ready ISR */
interrupt [ADC_INT] void adc_interrupt(void)
{
	converted = 1;
}
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------         
 /* main controls everything */        
void main (void)    
{          
	initialize();	// initialize  
	Vold = performConversion(); //starting value
	//printf("Vfirst: %d\r\n",Vold);
	
	while(1)     
	{    
		if (time1==0)
		{
			task1();       
		}
	    
	     if(scan_time == 0 && readyToConvert)
	    {     
	    
	    	scan_time = SCAN_INTERVAL;
	    
		if(mode == NOBREEZE)
		{     
		 	Vold = Vnew;
		  	Vnew = performConversion();
		  	deltaV = Vold - Vnew;
		  	if(deltaV >= MIN_V_CHANGE)
		  	{   
		  		printf("deltaV: %d\r\n",deltaV);	    
		  		prevMode = mode;
		  		mode = BLOW;
		  	}
		   	else        
		  	{                       
		  		//"else" there's still no breeze
		  		prevMode = NOBREEZE;
			}	
		  }
		  
		  else if(mode == BLOW)
		  {                  
		  	printf("Blow\r\n");
	  		Vold = Vnew;		// gotta update the ADC!!
			Vnew = performConversion(); //still using old deltaV  
		       if(deltaV >= DV_DT_STRONG)
		       {   
		        	printf("deltaV: %d\r\n",deltaV);   
		        	prevMode = mode;
		         	mode = STRONG;
		         	strong();	// change to strong breeze LED parameters
		         	if(deltaV >= BLOWOUT)
		         	{   
		         		printf("deltaV: %d\r\n",deltaV);
		         		printf("BLOWNOUT");
		         		DDRD = 0x00; //set PORTB to be all inputs
		         		PORTB = 0xff;//and turn them off
		         		//and Shut off interrupts, candle is out
		       			#asm
						cli
						#endasm
		         	}
		        }
		        else
		        {   
		        	printf("deltaV: %d\r\n",deltaV); 
			        prevMode = mode;
		        	mode = LIGHT;
		        	light();	// change to light breeze LED parameters
		        }
		  }
		  
		  else if(mode == LIGHT)
		  {                 
		  	
			Vold = Vnew;
			Vnew = performConversion();
			deltaV = Vold - Vnew;
			
	        if(prevMode != LIGHT) 
		        printf("LIGHT\r\n");
			
			if(deltaV >= DV_DT_STRONG)
		 	{    
		       	printf("deltaV: %d\r\n",deltaV);   
		       	prevMode = mode;
		       	mode = STRONG;
		       	strong();
		 	if(deltaV >= BLOWOUT)
		   	{  
		   		printf("deltaV: %d\r\n",deltaV);
				printf("BLOWNOUT");
		   		DDRD = 0x00; //set PORTB to be all outputs
		   		PORTB = 0xff;//and turn them off
		   		//and Shut off interrupts, candle is out
		   		#asm
		   		cli
		   		#endasm
		   	}
		        }
		        else
		        {   
		        	printf("deltaV: %d\r\n",deltaV);
		        	prevMode = mode;
		        	mode = LIGHTHELPER; 
		        }  
		        		        	
		  }
		  else if(mode == LIGHTHELPER)
		  {
		  	printf("Lighthelper\r\n");
		  	Vold = Vnew;		// gotta update the ADC!!
			Vnew = performConversion(); //still using old deltaV   
			if(deltaV >= MIN_V_CHANGE)
		  	{   
		  		printf("deltaV: %d\r\n",deltaV);    
		  		prevMode = mode;
		  		mode = LIGHT;
		  		//light();		//previous mode has to be light breeze, no don't need to reload
		  	}     
		  	else
		  	{   
		  		printf("deltaV: %d\r\n",deltaV);
		  	 	prevMode = mode;
		  	 	mode = NOBREEZE;
		  	 	nobreeze();
		  	}
			
		  	
		  }
		  
		  else if(mode == BLOWING)
		  {     
		  	printf("Blowing\r\n");
	  		Vold = Vnew;		// gotta update the ADC!!
			Vnew = performConversion(); //still using the old deltaV  
		  	if(deltaV >= MIN_V_CHANGE)
		        {    
		        	printf("deltaV: %d\r\n",deltaV);   
		        	prevMode = mode;
		         	mode = LIGHT;
		         	light();	// change to light breeze LED parameters
		        }
		        else
		        {   
		        	printf("deltaV: %d\r\n",deltaV); 
		        	prevMode = mode;
		        	mode = NOBREEZE;
		        	nobreeze();	// change to no breeze LED parameters
		        }	
		        	
		  }
		  
		  else if(mode == STRONG)
		  {                    
		  	Vold = Vnew;
			Vnew = performConversion();
			deltaV = Vold - Vnew;
		  	if(prevMode != STRONG)
		   	{                        
		   		printf("deltaV: %d\r\n",deltaV);
				printf("STRONG\r\n");
    		}       
		   		  	
		  	if(deltaV < DV_DT_STRONG)
		   	{   
		   		printf("deltaV: %d\r\n",deltaV);
		       	prevMode = mode;
		       	mode = BLOWING;
		    }
		     
		   	if(deltaV >= BLOWOUT)
		   	{   
		   		printf("deltaV: %d\r\n",deltaV);
				printf("BLOWNOUT");
		   		DDRD = 0x00; //set PORTB to be all outputs
		   		PORTB = 0xff;//and turn them off
		   		//and Shut off interrupts, candle is out
		   		#asm
		   		cli
		   		#endasm
		   	}
		    
		    //else it's (still) a strong breeze
		  }    
		  else
		  {
			printf("Problem!\r\n");
		  }	  
		  
	    } 
	    
	}// end while    		    
	
}// end main()
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
/*** conversion function ***/
int performConversion(void)
{
	ADMUX = 0x00;  // select bit 0 to convert
	ADCSR.6 = 1;   // start conversion   
 	
 	while(converted != 1);   // spin until conversion is done
	
	oldvoltage = voltage;	//save old value  
  	// read in value
	converted = 0;
	voltage = ADCL;
	upper = ADCH;
	upper = upper << 8;
	voltage = upper | voltage;
	if(oldvoltage != voltage)
	{
		printf("voltage is %d\r\n", voltage);
	}
	return voltage;    
}    

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

//Task 1
void task1(void) 
{   
	PORTB.1 = 0;
  	time1 = t1;  				//reset the task timer  
  	
  	next = next * 1103515245 + 12345;
	randomNum = (unsigned int)(next/65536) % 32768;
	finalRandomNum = randomNum;
	finalRandomNum = finalRandomNum/32767 - 0.5;   
	
	z_t1 = (z_t * alpha) + (beta * finalRandomNum);
	z_t  = z_t1;   
	putInOCR1A = (z_t * I_scale) + I_offset + (blow * finalRandomNum);
  
}

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
/* initialize everything */  
void initialize(void) 
{             
	/* setup the serial UART */
	UCR = 0x10 + 0x08 ;
	UBRR = 25 ;        
 
	/* setup the timer 0 */     
	/* 62.5 x (64 x0.25) microSec = 1.0 mSec, so prescale 64, and count 62 times. */ 
	timer0_reload = 256-62;		// value for 1 ms    
	TCNT0 = timer0_reload;		// preload timer 0 so that it interrupts after 1 ms  
	TCCR0 = PreScal0;			// prescalar to 64    
	TIMSK = 1;    
	
	//set up timer 1
	TIMSK  = 0b00000100 | 1;	  							//enable timer1 ovfl intr
 	TCCR1B = PreScal1;	  					       		//prescale timer1	
    TCCR1A = PWMnorm+PWM8bit; 				       			//8-bit pwm, output A normal         
    	
 	/*setup the ports*/
  	DDRD  = 0xff;             //set to all ones so that PORT D = all outputs
	PORTD = 0;                
	DDRB  = 0xff;
	PORTB = 0;                 //Lights on
    
	//PORT A initializations
	//initialize portA to be input
	DDRA = 0xfe;
	PORTA = 0xfe;  
             
	/* ADC stuff */
	// set ADC Enable (bit 7), in single conversion mode, enable interrupt flag
	// 1000 1110  // prescale 64  
	ADCSR = 0x8e;          
  
	readyToConvert = 1;
	converted = 0;
    
	/* interval timing stuff */
	scan_time = SCAN_INTERVAL; 
	
	//init the task timer
  	time1 = t1;
	              
	//control operation mode stuff
	mode = NOBREEZE;
	prevMode = NOBREEZE;
	
	//printf("Mode:  %d\r\n",mode);
	//printf("pMode: %d\r\n",prevMode);
	
	Vold = 0;
	Vnew = 0; 
     
	oldvoltage = 0;
	voltage = 0;  
		
	e_to_minus_one = eToMinusOne;
	tau = TAU;
 	Iscale  = I_scale;
	Ioffset = I_offset;
	Blow    = blow;

    deltat = t1/1000.00;
	exponent = deltat/tau; 
	alpha = 0;									//All white noise
	beta  = 1 - alpha;
    	        
 	next = 1;  
	finalRandomNum = 0;
    putInOCR1A =0;
    z_t  = 0;
    z_t1 = 0;
   	
	printf("initialized\r\n");
 	#asm 
	sei 
	#endasm
}   
