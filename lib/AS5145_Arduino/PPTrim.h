#include "Setup.h"

void pptrimRead(unsigned char *buffer, unsigned char num_bits)

{

        xdata unsigned char current_byte = 0;

        xdata unsigned char current_bit = 0;

        xdata unsigned char temp = 0;

        if(!num_bits) return;

        current_byte = num_bits >> 3;

        current_bit = num_bits & ~0x07;

        setupCondition();

        operationModeRead();

        clkPulses(1); // position the first bit to read

        //-- read OTP Data --

        temp = 0;

        temp += (PROG) ? 1 : 0;

        for(current_bit = num_bits; current_bit; current_bit--)

        {

                if(((current_bit - 1) & 0x07) == 0)

                {

                        buffer[current_bit >> 3] = temp;

                        temp = 0;

                }

                if (current_bit)

                {

                        temp <<= 1;

                        SET_CLK();

                        pptrimDelay(200);

                        temp += (PROG) ? 1 : 0;

                        CLEAR_CLK();

                        pptrimDelay(200);

                }

        }

        exitCondition();

}



void pptrimLoad(unsigned char num_bits)
{
	setuCondition();
	operationModeLoad();
	clkPulses(num_bits);
	exitCondition();
}



void pptrimWrite(unsigned char *buffer, unsigned char num_bits)

{

        xdata unsigned char *current_byte;

        xdata unsigned char current_bit = 0;

        xdata unsigned char temp = 0;

        current_byte = buffer + ((num_bits-1)>>3);

        temp = *current_byte;

        if(num_bits % 8)

        temp <<= 8 - (num_bits % 8);

        setupCondition();

        operationModeWrite();

        //-- send OTP Data

        for(current_bit = num_bits; current_bit; current_bit--)

        {

                if(temp & 0x80)

                SET_PROG();

                else

                CLEAR_PROG();

                pptrimDelay(100);

                SET_CLK();

                pptrimDelay(300);// delay, tzapp=2us(typ.)

                CLEAR_CLK();

                pptrimDelay(PPTRIMDelay);

                temp <<= 1;

                if(((current_bit-1) & 0x07) == 0)

                {

                        temp = *(--current_byte);

                }

        }

        SET_PROG();

        pptrimDelay(100);

        clkPulses(1); // data latched

        // END OTP-Write

        exitCondition();

}
