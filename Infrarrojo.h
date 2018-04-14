class Infrarrojo {  
public:
  Infrarrojo (int p,int b); 
  bool lee_blanco();
  bool lee_negro();

private:
  int pin;  
  int blanco;
};

Infrarrojo::Infrarrojo(int p, int b)
{
  pin = p;
  blanco = b;
  pinMode(pin,INPUT);
}

bool Infrarrojo::lee_blanco()
{
  if(analogRead(pin) >= blanco)
    return true;
 else return false;
}

bool Infrarrojo::lee_negro()
{
  if(analogRead(pin) < blanco)
    return true;
   else return false;
}

