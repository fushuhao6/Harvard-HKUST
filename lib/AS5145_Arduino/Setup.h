#include "Utility.h"


static void setupCondition()
{
	CLEAR_CSN();
	pptrimDelay(PPTRIMDelay);
	CLEAR_CLK();
	pptrimDelay(PPTRIMDelay);
	SET_PROG();
	pptrimDelay(PPTRIMDelay);
	SET_CSN();
	pptrimDelay(PPTRIMDelay);
	CLEAR_CSN();
	pptrimDelay(PPTRIMDelay);
	SET_CLK();
	pptrimDelay(PPTRIMDelay);
	CLEAR_CLK();
	pptrimDelay(PPTRIMDelay);
}
	

static void exitCondition()
{
	PROG_LOW_IMPED();
	pptrimDelay(PPTRIMDelay);
	CLEAR_CSN();
	pptrimDelay(PPTRIMDelay);
	SET_CLK();
	pptrimDelay(PPTRIMDelay);
	CLEAR_CLK();
	pptrimDelay(PPTRIMDelay);
	SET_CLK();
	pptrimDelay(PPTRIMDelay);
	SET_CSN();
	pptrimDelay(PPTRIMDelay);
	CLEAR_PROG();
	pptrimDelay(PPTRIMDelay);
}
	

static void operationModeLoad()
{
	CLEAR_PROG();

        pptrimDelay(PPTRIMDelay);

 	SET_CSN();

	pptrimDelay(PPTRIMDelay);

	clkPulses(1);
}



static void operationModeRead()

{

        CLEAR_PROG();

        pptrimDelay(PPTRIMDelay);

        SET_CLK();

        pptrimDelay(PPTRIMDelay);

        SET_CSN();

        pptrimDelay(PPTRIMDelay);

        CLEAR_CLK();

        pptrimDelay(PPTRIMDelay);

        clkPulses(1);

        PROG_HIGH_IMPED();

}



static void operationModeWrite()

{

        SET_CSN();

        pptrimDelay(PPTRIMDelay);

        clkPulses(3);

}

        
