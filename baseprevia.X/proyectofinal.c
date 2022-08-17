/*
 * File:   proyectofinal.c
 * Author: ivanm
 *
 * Created on 25 de junio de 2022, 07:13 PM
 */
#include<xc.h>
#define _XTAL_FREQ 8000000
#include "LibLCDXC8.h"
#pragma config FOSC=INTOSC_EC
#pragma config WDT=OFF
#pragma config PBADEN=OFF
#pragma config LVP=OFF
#define DATA_DIR TRISC2
#define F_X RB0
#define F_Y RB1
#define F_Z RB2
#define dir_X LATE0
#define dir_Y LATE1
#define dir_Z LATE2
#define step_X LATB3
#define step_Y LATB4
#define step_Z LATB5
const int stepDelay = 1;
int vactual=0,vs=0,carro=0,contador=0;
const int piso1=1000;//950;
const int piso2=1500;//1450;
const int piso3=2000;//1950;
const int piso4 =2360;
const int columna1 = 0;
const int columna2 = 1100;
const int columna3 = 2200;
int carro1[2] ={0,2000};
int carro2[2] ={1100,2000};
int carro3[2] ={2200,2000};
int carro4[2] ={0,1500};
int carro5[2] ={1100,1500};
int carro6[2] ={2200,1500};
int carro7[2] ={0,1000};
int carro8[2] ={1100,1000};
int carro9[2] ={2200,1000};
//estadob 0=recogido 1= extendido
//estado 0=calibracion 1=esperando ordenes 2=esperando carro 3=guardandocarro 4=sacando carro
int estado =0, estadob=0;
unsigned char Che,lectura;
unsigned long long int posX,posY,PosZ,auxT;
unsigned char Dec,Uni;
void Transmitir(unsigned char);
void setorigin(void);
void ira(int,int);
void extender(void);
void recoger(void);
void guardarV();
void sacarV();
unsigned char Recibir(void);
void __interrupt() ISR(void);
void main(void){
    ADCON1=0b00001111;//se definen todos los pines como digitales
    LATE=0X00;
    TRISE=0X00;
    //ADCON1bits.PCFG=0xFF;
    LATB=0X00;
    TRISB=0b00000111;
    OSCCON=0b01110000; //CAMBIO FRECUENCIA 8MHZ
    RBPU=0;
    TRISD=0;
    LATD=0; 
    __delay_ms(1000);
    //config LCD
    ConfiguraLCD(4);
    InicializaLCD();
    MensajeLCD_Var("Parqueadero");
    //config comunicacion
  //  TXSTA=0b00100100;
    RCSTA=0b10010000;
    TXSTA=0b00100000;
    //BAUDCON=0b00001000;
    BAUDCON=0b00000000;
    SPBRG=12;
    //SPBRG=25;
    //interrupcion para temporal
    T0CON=0b10000100;
    TMR0=3036;
    TMR0IF=0;
    TMR0IE=1;
    //TMR0ON=1;
    
    //interupcion recepcion de datos
    RCIP=1;
    RCIF=0;
    RCIE=1;
    GIE=1;
   // CREN=1;
  setorigin();
  //Transmitir('o');
  //Transmitir('k'); 
  __delay_ms(5000);
  BorraLCD();
  MensajeLCD_Var("listo");

  while(1){
      if(estado ==0){
      MensajeLCD_Var("Set origin");
      setorigin();
      }
      else if(estado==3){
          BorraLCD();
          MensajeLCD_Var("guardando");
          DireccionaLCD(0xC0);
          MensajeLCD_Var("Carro:");
          EscribeLCD_n8(vactual, 1);
          guardarV();
      }
      else if (estado==4){
          BorraLCD();
          MensajeLCD_Var("sacando");
          DireccionaLCD(0xC0);
          MensajeLCD_Var("Carro:");
          EscribeLCD_n8(carro, 1);
          sacarV();
      }
      else{
       BorraLCD();
       MensajeLCD_Var("Espera de orden");
       __delay_ms(1000);
      }
  }
}
void Transmitir(unsigned char dato){
    //se configura momentaneamente para envio
    while(TRMT==0);
    TXREG=dato;
}
void setorigin(void){
  dir_X = 1;
  dir_Y = 1;
  dir_Z = 0;
  while(F_Z==1){
        if(F_Z == 1)step_Z=1;
        __delay_ms(stepDelay);
        step_Z=0;
        __delay_ms(stepDelay);
  }
  while(F_X == 1 || F_Y == 1){
    if(F_X == 1)step_X=1;
    if(F_Y == 1)step_Y=1;
      __delay_ms(stepDelay);
      step_X=0;
      step_Y=0;
      __delay_ms(stepDelay);
    }
    if(F_X == 0){
        posX = 0;
    }
    if(F_Y== 0){
        posY = 0;
    }
    dir_Z = 1;
   for(int i=0;i<800;i++){//500
      step_Z = 1;
      __delay_ms(stepDelay);
      step_Z =0;
      __delay_ms(stepDelay);
      }
    estadob=0;
    estado=1;
    vactual=0;
}
void ira(int desX,int desY){
 if(desX < posX)dir_X= 1;
  else dir_X = 0;
   if(desY < posY)dir_Y = 1;
  else dir_Y = 0;
  while(desX != posX || desY != posY){
    if( desX != posX){
       step_X= 1;
      if(desX < posX)posX--;
      else posX++;
    }
    if( desY != posY){
      step_Y = 1;
      if(desY < posY)posY--;
      else posY++;
    }
      __delay_ms(stepDelay);
      step_X=0;
      step_Y=0;
      __delay_ms(stepDelay);
    }

  }
//Mete el brazo para tomar la estiba
void extender(){
    dir_Z = 1;
   if(estadob==0){
   for(int i=0;i<1500;i++){
      step_Z = 1;
      __delay_ms(stepDelay);
      step_Z = 0;
      __delay_ms(stepDelay);
      }
   estadob=1;
  }
}

  //Saca el brazo para sacar la estiba
void recoger(){
  dir_Z = 0;
  if(estadob==1){
   for(int i=0;i<1500;i++){
      step_Z = 1;
      __delay_ms(stepDelay);
      step_Z =0;
      __delay_ms(stepDelay);
      }
  }
  estadob=0;
  
}
void sacarV(){
    vs=carro;
    if(vactual==0){
        recoger();
        if(vs==1){
            ira(carro1[0],carro1[1]);
            vactual=1;
        }
        else if(vs==2){
            ira(carro2[0],carro2[1]);
            vactual=2;
        }
        else if(vs==3){
            ira(carro3[0],carro3[1]);
            vactual=3;
        }
        else if(vs==4){
            ira(carro4[0],carro4[1]);
            vactual=4;
        }
        else if(vs==5){
            ira(carro5[0],carro5[1]);
            vactual=5;
        }else if(vs==6){
            ira(carro6[0],carro6[1]);
            vactual=6;
        }else if(vs==7){
            ira(carro7[0],carro7[1]);
            vactual=7;
        }else if(vs==8){
            ira(carro8[0],carro8[1]);
            vactual=8;
        }else if(vs==9){
            ira(carro9[0],carro9[1]);
            vactual=9;
        }
        ira(posX,posY-50);
        extender();
        ira(posX,posY+50);
        recoger();
        ira(columna2,piso3);
        ira(columna2,piso4);
        vs=0;
      }
    else{
        Transmitir('E');
        Transmitir('R');
        Transmitir('R');
        Transmitir('O'); 
        Transmitir('R');
        Transmitir(' ');
        Transmitir('G');
        Transmitir('U');
        Transmitir('A');
        Transmitir('R'); 
        Transmitir('D');
        Transmitir('E');
        Transmitir(' ');
        Transmitir('A');
        Transmitir('N');
        Transmitir('T');
        Transmitir('E'); 
        Transmitir('S');
    }
    estado=1;
}
void guardarV(){
    if(vactual!=0){
        ira(posX,posY-100);
        if(vactual==1){
            ira(carro1[0],carro1[1]);
        }
        else if(vactual==2){
            ira(carro2[0],carro2[1]);
        }
        else if(vactual==3){
            ira(carro3[0],carro3[1]);
        }
        else if(vactual==4){
            ira(carro4[0],carro4[1]);
        }
        else if(vactual==5){
            ira(carro5[0],carro5[1]);
        }else if(vactual==6){
            ira(carro6[0],carro6[1]);
        }else if(vactual==7){
            ira(carro7[0],carro7[1]);
        }else if(vactual==8){
            ira(carro8[0],carro8[1]);
        }else if(vactual==9){
            ira(carro9[0],carro9[1]);
        }
        extender();
        ira(posX,posY-50);
        recoger();
        vactual=0;
      }
    else{
        Transmitir('E');
        Transmitir('R');
        Transmitir('R');
        Transmitir('O'); 
        Transmitir('R');
        Transmitir(' ');
        Transmitir('S');
        Transmitir('A');
        Transmitir('Q');
        Transmitir('U'); 
        Transmitir('E');
        Transmitir(' ');
        Transmitir('A');
        Transmitir('N');
        Transmitir('T');
        Transmitir('E'); 
        Transmitir('S');
    }
    estado=1;
  }
void __interrupt() ISR(void){
    if(RCIF==1){
       lectura=RCREG;
       contador=0;
       if(estado==1){
       Transmitir(lectura);
       if(lectura=='1')carro=1;
       if(lectura=='2')carro=2;
       if(lectura=='3')carro=3;
       if(lectura=='4')carro=4;
       if(lectura=='5')carro=5;
       if(lectura=='6')carro=6;
       if(lectura=='7')carro=7;
       if(lectura=='8')carro=8;
       if(lectura=='9')carro=9;
       if(lectura=='G'||lectura=='g')estado=3;
       if(lectura=='S'||lectura=='s')estado=4;
       if(lectura=='H'||lectura=='h')estado=0;
       }
       RCIF=0;
    }
    if(TMR0IF==1){
        TMR0IF=0;
        if(estado==1){
            contador++;
        }
        if(contador>80 && vactual==0 ){
            BorraLCD();
            MensajeLCD_Var("calibracion");
            DireccionaLCD(0xC0);
            MensajeLCD_Var("Rutinaria");
            contador=0;
            setorigin();
        }
        else if(contador>100){
            BorraLCD();
            MensajeLCD_Var("guardando por:");
            DireccionaLCD(0xC0);
            MensajeLCD_Var("inactividad");
            contador=60;
            guardarV();
        }

        TMR0=3036;
    }
}