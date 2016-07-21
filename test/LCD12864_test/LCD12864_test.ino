
#include "LCD12864.h"

const uint8_t LED_G = 13;
bool state = HIGH;

void setup()
{
  Serial.begin(9600);

  Serial.println("Hello");
  
  pinMode(LED_G, OUTPUT);
  digitalWrite(LED_G, state);
  LCDA.Initialise(); //@Initialize
  delay(500);
  LCDA.Render();

  LCDA.DrawCircle(30,135,5);
  LCDA.RenderScreenBuffer(2); // lets draw it in the second screen


}
 
void loop()
{
  state = !state;
  digitalWrite(LED_G, state);

  LCDA.Draw(false,4,0);
  delay(1);
  LCDA.setPins(1,0,0,0,1,1,0,0,0,0);

  LCDA.setPins(1,0,0,0,1,1,0,0,0,1);

  LCDA.setPins(1,0,0,0,1,1,0,0,1,0);

  LCDA.Draw(true,4,0);
  delay(200);
}
