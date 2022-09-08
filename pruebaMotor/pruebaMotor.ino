int dir = 8;
int stp = 9;

int pasos = 8700; //17600
int pasosDados = 0;
//int tp = 210;
int i = 0;
int j = 0;
int contador;

int tbase = 4; //PAra tener 4 segundos de aceleracion [s]
float Tvc = 4200; //Si se cambia lo anterior usar excel para cambiar este parametro [ms]

int deltaT = 200;//(tbase / 20) * 1000; //200 milis cada intervalo
float tp; //Delay para el tiempo

float Vc; //Velocidad crucero
float Vins = 0; //Velocidad en cada intervalo

unsigned long prevMillis = 0;
unsigned long currMillis = millis();

//PRUEBA PLIS BORRAR ESTO
unsigned long a = 0;
unsigned long b = 0;
unsigned long c = 0;
//-----------------

void movimiento() {
  Vc = pasos * 0.5 / tbase;
  acelerar();
  crucero();
  desacelerar();
}

void darPaso();

void acelerar() {
  currMillis = millis();
  prevMillis = currMillis;
  contador = 1;
  Vins = (contador * 1.0 / 20) * Vc;
  tp = (unsigned long)((1 / (2 * Vins)) * (1000000));
  int j = 0;
  while (contador < 20) {
    currMillis = millis();
    if (currMillis - prevMillis < deltaT) {
      darPaso();
    } else if (currMillis - prevMillis >= deltaT) {
      contador++;
      prevMillis = currMillis;
      Vins = (contador * 1.0 / 20) * Vc;
      tp = (unsigned long)((1 / (2 * Vins)) * (1000000));
    }
    j++;
  }
}

void crucero() {
  currMillis = millis();
  prevMillis = currMillis;
  tp = tp = (1 / (2 * Vc)) * (1000);
  while (currMillis - prevMillis < Tvc) {
    darPaso();
  }
}

void desacelerar() {
  currMillis = millis();
  prevMillis = currMillis;
  contador = 19;
  while (contador > 0) {
    currMillis = millis();
    if (currMillis - prevMillis < deltaT) {
      darPaso();
    } else if (currMillis - prevMillis >= deltaT) {
      contador--;
      prevMillis = currMillis;
      if (contador != 0) {
        Vins = (contador / 20) * Vc;
        tp = (1 / (2 * Vins)) * (1000);
      }
    }
  }
  while (pasosDados < pasos) {
    darPaso();
  }
}

void setup() {
  pinMode(dir, OUTPUT);
  pinMode(stp, OUTPUT);
  digitalWrite(dir, LOW);
  i = 0;
  Vc = pasos * 0.5 / tbase;
  contador = 10;
  Vins = (contador * 1.0 / 20) * Vc;
  tp = (unsigned long)((1 / (2 * Vins)) * (1000000));
  a = 0;
  prevMillis = 0;
  b = 5001514524862;
}

void loop() {

  /*if(i<1000){
    darPaso();
    i++;
    }*/

  if (a < deltaT) {
    //Vc = pasos * 0.5 / tbase;
    //acelerar();
    darPaso();
    a = millis() - prevMillis;
    i++;
  } else {
    if (contador < 21 && i != 145) {
      contador++;
      prevMillis = millis();
      a = 0;
      i = 0;
    } else if (b < Tvc) {
      darPaso();
      b = millis() - prevMillis;
      i = 145;
      contador = 19;
    } else if (i != 145) {
      tp = tp = (1 / (2 * Vc)) * (1000);
      prevMillis = millis();
      b = millis() - prevMillis;
    } else if (b >= Tvc && i == 145 && j != 450) {
      prevMillis = millis();
      j = 450;
      c = 0;
    } else if (c < deltaT) {
      darPaso();
      c = millis() - prevMillis;
    } else {
      if (contador > 0) {
        contador--;
        prevMillis = millis();
        c = 0;
      } else {
        if (pasosDados < pasos) {
          darPaso();
        }
      }
    }
    if (contador != 0) {
      Vins = (contador * 1.0 / 20) * Vc;
      tp = (unsigned long)((1 / (2 * Vins)) * (1000000));
    }

  }
}

void darPaso() {
  digitalWrite(stp, HIGH);
  delayMicroseconds(tp);
  digitalWrite(stp, LOW);
  delayMicroseconds(tp);
  pasosDados++;
}
