#define PPTRIMDelay 50

#define CLEAR_CSN()     do { CSN = 0; } while(0) 
#define SET_CSN()       do { CSN = 1; } while(0) 
#define CLEAR_CLK()     do { CLK = 0; } while(0) 
#define SET_CLK()       do { CLK = 1; } while(0) 
#define CLEAR_PROG()    do { PROG = 0; } while(0) 
#define SET_PROG()      do { PROG = 1; } while(0) 
#define PROG_HIGH_IMPED() do { P0MDOUT   = 0x5F; PROG = 1;} while(0) // Pushpull disabled, Hi-Z 
#define PROG_LOW_IMPED() do { P0MDOUT   = 0xDF; } while(0) // Pushpull enabled 

static void pptrimDelay(volatile unsigned int value)
{        
	for(value; value>0; value--);
        {          
		unsigned char foo = 30;
          	while(foo--);        
	} 
}

void initPPTRIM() 
{        
	PROG_LOW_IMPED();
	        CLEAR_PROG(); 
}


static void clkPulses(unsigned char num) {   
	unsigned char i;   
	for(i = 0; i < num; i++)   
	{
	        SET_CLK();     
		pptrimDelay(PPTRIMDelay); 	
		CLEAR_CLK();   
		pptrimDelay(PPTRIMDelay);   
	} 
} 


unsigned char reversebits(unsigned char value) // Endian switch 
{        
	unsigned char i=0, result=0;        
	while (i<8)        
	{               
		result += (value<<i)&0x80;
               if (i<7) result = result >> 1;               
		i++;        
	}        
	return result; 
}

