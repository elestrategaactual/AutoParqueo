/*
 * File:   aceleration.c
 * Author: felip
 *
 * Created on 9 de septiembre de 2022, 02:04 PM
 */


#include <xc.h>
#pragma config FOSC=INTOSC_EC
#define _XTAL_FREQ 8000000
#pragma config WDT = OFF
#pragma config PBADEN = OFF
#pragma config LVP=OFF

void __interrupt() ISR(void);
//unsigned int array(int pasos);
void precargas(void);

int pasosX,pasosY,pasosZ;        //Pasos requeridos
int pasosXDados,pasosYDados,pasosZDados;        //Pasos dados
float Ta; //Tiempo de aceleracion y desaceleracion [s]
float deltaT;     //Tiempo de cada intervalo  [s]
float vCx,vCy;        //Velocidad crucero [pasos/s]
float vInsx,vInsy;    //Velocidad instantanea [pasos/s]
float TIntx,TInty;    //Periodo de Interrupción[s]
float v5x,v5y;          //Velocidad del 5%
unsigned int precargaX,precargaY;        //Precarga para las interrupciones
float TvC;    //Tiempo de velocidad crucero [s]
int ac,cru,des,counter,counterVC;
unsigned int arrayX[20];        //Precargas para X
unsigned int arrayY[20];        //Precargas para Y


void main(void) {
    
    counter = -1;
    counterVC = 0;
    ac=1;
    des=0;
    cru=0;
    
    pasosX = 7000;
    pasosY = 10000;
    pasosXDados = 0;
    pasosYDados = 0;
    Ta = 4.5; //s
    deltaT = Ta/20;//s
    vCx = (pasosX*0.5)/Ta;
    vCy = (pasosY*0.5)/Ta;
    v5x = 0.05*vCx;
    v5y = 0.05*vCy;
    
    precargas();
    
    //Configuramos tres pines del puerto D como salida para los pulsos de los tres motores
    TRISD = 0b11100000; //D0 xStep, D1 YStep, D2 xDir, D3 yDir
    LATD = 0;
    
    //Direcciones
    LATD2 = 1;      //Dirección motor X
    LATD3 = 1;      //Dirección motor Y
    
    //Limpiamos las banderas de los temporizadores
    TMR0IF = 0;
    TMR1IF = 0;
    TMR3IF = 0;
    
    //Habilitamos las interrupciones de los temporizadores
    TMR0IE = 1;
    TMR1IE = 1;    
    TMR3IE = 1;
    
    //Frecuencia de oscilador en 8 MHz
    OSCCON=0b01110000;  
    
    
    //Configuración TMR0
    T0CON = 0b00000010;
    TMR0ON = 1;
    TMR0 =9286;        //Interrupciones cada 0.225 s
    
    //Configuración TMR1
    T1CON = 0b10100000;
    TMR1ON = 1;
    
    //Configuración TMR3
    T3CON = 0b10100000;
    TMR3ON = 1;
    
    
    //Habilitamos las interrupciones perifericas
    PEIE = 1;
    
    //Habilitamos todas las interrupciones
    GIE = 1;
    
    while(1){}
}

void __interrupt() ISR(void){
    if(TMR0IF==1){
        TMR0IF=0;
        TMR0 = 9286;
        
        if (ac==1){
            counter++;
        } else if (des==1){
            counter--;
        } else if (cru==1){
            counterVC++;
        }
        
        if(counter>=19 && ac==1){
            ac=0;
            cru=1;
        }else if(counter>=19 && cru==1 && counterVC>=20){
            cru=0;
            des=1;
        }else if(counter<0  && des==1){
            des=0;
        }
        
        if(counter>=0 && counter<=19){
            precargaX=arrayX[counter];
            precargaY=arrayY[counter];
        }
        
        if(pasosXDados<pasosX){
            //TMR1ON=1;
            TMR1 = precargaX;
        }
        
        if(pasosYDados<pasosY){
            //TMR3ON=1;
            TMR3 = precargaY;
        }  
    }
    
    if(TMR1IF==1){
        TMR1IF=0;
        TMR1 = precargaX;
        
        if(LATD0==1 && pasosXDados<pasosX){
            pasosXDados++;
            LATD0=0;
        } else if(LATD0==0 && pasosXDados<pasosX){
            LATD0=1;
        }
        
    }
    
    if(TMR3IF==1){
        TMR3IF=0;
        TMR3 = precargaY;
        
        if(LATD1==1 && pasosYDados<pasosY){
            pasosYDados++;
            LATD1 = 0;
        } else if(LATD1==0 && pasosYDados<pasosY){
            LATD1=1;
        }
    }
}

void precargas(void){
   int i;
   for (i=1;i<21;i++){
        vInsx = i*v5x;//(1.0*counter*vCx)/20.0;
        vInsy = i*v5y;//(1.0*counter*vCy)/20.0;
        TIntx = 0.5/vInsx;//1.0/(vInsx*2.0);
        TInty = 0.5/vInsy;//1.0/(vInsy*2.0);
        arrayX[i-1] = (unsigned int)(65536 - ((TIntx*2000000)/4));//(unsigned int)(65536 - ((TIntx*2000000)/4));
        arrayY[i-1] = (unsigned int)(65536 - ((TInty*2000000)/4));
   }
}
