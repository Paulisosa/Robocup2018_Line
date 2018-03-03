const int blanco = 200;
const int nsensores = 7;
int pines[nsensores] = {1,2,3,4,5,6,7};

void setup() {  
  Serial.begin(9600);
  
  for(int i = 0; i < nsensores; i++)
      pinMode(pines[i],INPUT);
}

bool lee_blanco(int pin)
{
      if(analogRead(pines[pin]) < blanco) 
         return false;

    return true;
}

void loop() {
  for(int i = 0; i < nsensores; i++)
  {
    print("Sensor ");
    print(i);
    print(": ");
    if(lee_blanco(i))
      println("Lee blanco");
     else
     println("No lee blanco");
  }
    

  delay(2000);
}
