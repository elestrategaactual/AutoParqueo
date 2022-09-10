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
void precargas(void);
void goHome(void);
void mover(long int x,long int y, long int z,int absoluto);

const int tDelay = 400;
unsigned int pasosX,pasosY,pasosZ;        //Pasos requeridos
unsigned int pasosXDados,pasosYDados,pasosZDados;        //Pasos dados
float Ta; //Tiempo de aceleracion y desaceleracion [s]
float deltaT;     //Tiempo de cada intervalo  [s]
float vCx,vCy;        //Velocidad crucero [pasos/s]
float vInsx,vInsy;    //Velocidad instantanea [pasos/s]
float TIntx,TInty;    //Periodo de Interrupción[s]
float v5x,v5y;          //Velocidad del 5%
unsigned int precargaX,precargaY;        //Precarga para las interrupciones
float TvC;    //Tiempo de velocidad crucero [s]
int ac,cru,des;                 //Banderas de proceso
int finX,finY,finZ;             //Banderas de finalización
int counter,counterVC;          //Contadores
unsigned int arrayX[20];        //Precargas para X
unsigned int arrayY[20];        //Precargas para Y
unsigned int posiciones[11][2]={{0,0},                      //Home
                                {0,15600},                  //car1
                                {8700,15600},               //car2
                                {17600,15600},              //car3
                                {0,11600},                  //car4
                                {8700,11600},               //car5
                                {17600,11600},              //car6
                                {0,7600},                   //car7
                                {8700,7600},                //car8
                                {17600,7600},               //car9
                                {8700,19500}};              //StandBy - Posiciones absolutas
unsigned int posX,posY,posZ;        //Posiciones del robot


void main(void) {
    
    Ta = 4.5; //s
    deltaT = Ta/20;//s
    
    mover(posiciones[9][0],posiciones[9][1],0,0);
        
    //Configuramos tres pines del puerto D como salida para los pulsos de los tres motores
    TRISD = 0b111000000; //D0 xStep, D1 YStep, D2 xDir, D3 yDir, D4 zDir, D5 ZStep
    LATD = 0;
    
    //Direcciones
    LATD2 = 1;      //Dirección motor X
    LATD3 = 1;      //Dirección motor Y
    LATD4 = 1;      //Dirección motor Z
    
    //Configuracion pines de los finales de carrera
    TRISB = 0b11111111; //B0 FinalX, B1 FinalY, B2 FinalZ
    
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
        
        if(finX==0){
            TMR1 = precargaX;
        }
        
        if(finY==0){
            TMR3 = precargaY;
        }
        
        if(finY==1 && finX==1){
            TMR0ON = 0;
            TMR1ON = 0;
            TMR3ON = 0;
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
        }  else if(pasosXDados>=pasosX){
            finX = 1;
            TMR1ON = 0;
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
        } else if(pasosYDados>=pasosY){
            finY = 1;
            TMR3ON = 0;
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

void goHome(void){
    //Direcciones
    LATD2 = 0;      //Dirección motor X
    LATD3 = 0;      //Dirección motor Y
    LATD4 = 0;      //Dirección motor Z
    

    
    while(RB2==0){  //Devolver el Z
        LATD5=1;
        __delay_us(tDelay);
        LATD5=0;
        __delay_us(tDelay);
    }
    
    while(RB0==0 || RB1==0){
        if(RB0==0){
            LATD0=1;    //Step X
        }
        
        if(RB1==0){
            LATD1=1;    //Step Y
        }
        
        __delay_us(tDelay);
        
        LATD0=0;
        LATD1=0;
        
        __delay_us(tDelay);
    }
    
    posX=0;
    posY=0;
    posZ=0;
}


void mover(long int x,long int y, long int z, int absoluto){
    if(absoluto==1){    //Absoluto
        if(x>posX){
            pasosX = (unsigned int)(x-posX);
            LATD2 = 1;      //Dirección motor X
        }else{
            pasosX = (unsigned int)(posX-x);
            LATD2 = 0;      //Dirección motor X
        }
        
        if(y>posY){
            pasosY = (unsigned int)(y-posY);
            LATD3 = 1;      //Dirección motor y
        }else{
            pasosY = (unsigned int)(posY-y);
            LATD3 = 0;      //Dirección motor y
        }
        
        if(z>posZ){
            pasosZ = (unsigned int)(z-posZ);
            LATD4 = 1;      //Dirección motor Z
        }else{
            pasosZ = (unsigned int)(posZ-z);
            LATD4 = 0;      //Dirección motor Z
        }
    }else{      //Incremental
        if(x>0){
            pasosX = (unsigned int)(x);
            LATD2 = 1;      //Dirección motor X
        }else{
            pasosX = (unsigned int)(-x);
            LATD2 = 0;      //Dirección motor X
        }
        
        if(y>0){
            pasosY = (unsigned int)(y);
            LATD3 = 1;      //Dirección motor y
        }else{
            pasosY = (unsigned int)(-y);
            LATD3 = 0;      //Dirección motor y
        }
        
        if(z>0){
            pasosZ = (unsigned int)(z);
            LATD4 = 1;      //Dirección motor Z
        }else{
            pasosZ = (unsigned int)(-z);
            LATD4 = 0;      //Dirección motor Z
        }
    }
    
    if (pasosX>0){
        pasosXDados = 0;
        vCx = (pasosX*0.5)/Ta;
        v5x = 0.05*vCx;
    }
    
    if (pasosY>0){
        pasosYDados = 0;
        vCy = (pasosY*0.5)/Ta;
        v5y = 0.05*vCy;
    }
    
    if(pasosX>0 || pasosY>0){
        counter = -1;
        counterVC = 0;
        ac=1;
        des=0;
        cru=0;
        
        precargas();
    }        
}
