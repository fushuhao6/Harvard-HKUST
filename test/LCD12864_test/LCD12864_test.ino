
#include "LCD12864.h"

const uint8_t LED_G = 13;
bool state = HIGH;

void setup()
{
  pinMode(LED_G, OUTPUT);
  digitalWrite(LED_G, state);
  LCDA.Initialise(); //@Initialize
  delay(500);

  LCDA.Render();
  LCDA.clear();
}
 
void loop()
{
  state = !state;
  digitalWrite(LED_G, state);
//  LCDA.clear();
  LCDA.DrawCircle(30,135,5);
  LCDA.RenderScreenBuffer(2);

  LCDA.Draw(false,4,0);
//  delay(1);
//  LCDA.setPins(1,0,0,0,1,1,0,0,0,0);
//
//  LCDA.setPins(1,0,0,0,1,1,0,0,0,1);
//
//  LCDA.setPins(1,0,0,0,1,1,0,0,1,0);

  char* str = "NOT Hello World!!!";
  LCDA.DrawStr(str);
  LCDA.Draw(true,4,0);
  delay(1000);
}
