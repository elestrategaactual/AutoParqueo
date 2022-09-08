/*
 * File:   Aceleracion.c
 * Author: felip
 *
 * Created on 31 de agosto de 2022, 08:17 PM
 */


#include <xc.h>
#pragma config FOSC=INTOSC_EC
#pragma config WDT = OFF
#pragma config PBADEN = OFF
#pragma config LVP=OFF

int pasosX,pasosY,pasosZ;        //Pasos requeridos
int pasosXDados,pasosYDados,pasosZDados;        //Pasos requeridos
float Ta; //Tiempo de aceleracion y desaceleracion [s]
float deltaT;     //Tiempo de cada intervalo  [s]
float vCx,vCy;        //Velocidad crucero [pasos/s]
float vInsx,vInsy;    //Velocidad instantanea [pasos/s]
float TIntx,TInty;    //Periodo de Interrupción[s]
int precargaX,precargaY;        //Precarga para las interrupciones
float TvC;    //Tiempo de velocidad crucero [s]

int ac,cru,des,counter,counterVC;

void __interrupt() ISR(void);

void main(void) {
    
    pasosY=19500;        //Pasos requeridos en el eje Y
    pasosX = 17600;      //Pasos requeridos en el eje X
    pasosXDados = 0;    //Pasos dados X
    pasosYDados = 0;    //Pasos dados Y
    ac = 1;
    Ta = 4.5; //Tiempo de aceleracion y desaceleracion [s]
    deltaT = Ta/20.0;     //Tiempo de cada intervalo  [s]
    vCx = pasosX/Ta;        //Velocidad crucero eje x [pasos/s]
    vCy = pasosY/Ta;        //Velocidad crucero eje x [pasos/s]
    TvC = 21.0*deltaT;    //Tiempo de velocidad crucero [s]
    counter = 0;
    counterVC = 0;
    
    //Limpiamos las banderas de los temporizadores
    TMR0IF = 0;
    TMR1IF = 0;
    TMR3IF = 0;
    
    //Habilitamos las interrupciones de los temporizadores
    TMR0IE = 1;
    TMR1IE = 1;    
    TMR3IE = 1;
    
    
    //Configuración TMR0
    T0CON = 0b00000000;
    TMR0ON = 0;
    TMR0 = 65536 - ((deltaT*250000)/2);        //Interrupciones cada 0.225 s
    
    //Configuración TMR1
    T1CON = 0b10000000;
    TMR1ON = 0;
    
    //Configuración TMR3
    T3CON = 0b10000000;
    TMR3ON = 0;
    
    //Habilitamos todas las interrupciones
    GIE = 1;
    
    //Configuramos tres pines del puerto D como salida para los pulsos de los tres motores
    TRISD = 0b11111000;
    LATD = 0;
    
    //FALTA ENCENDER LOS TIMER LOL
    
    while(1){
    }
}

void __interrupt() ISR(void){
    if (TMR1IF == 1)   {
        if(pasosXDados < pasosX){
            LATD0 = !LATD0;
        }
        TMR1 = precargaX;
        if(LATD0==1){
            pasosXDados++;
        }
    }
    
    if (TMR3IF == 1){
        if(pasosYDados<pasosY){
            LATD1 = !LATD1;
        }
        TMR3 = precargaY;
        if(LATD1==1){
            pasosYDados++;
        }
    }
    
    if (TMR0IF == 1){
               
        if(ac == 1){
            counter+=1;
        }else if(des == 1){
            counter-=1;
        }else if(cru ==1){
            counterVC+=1;
        }
        
        if(counter>0 && (ac==1 || des == 1)){
            TMR1ON = 0;
            TMR3ON = 0;
            vInsx = (counter*vCx)/20.0;
            vInsy = (counter*vCy)/20.0;
            TIntx = 1/(vInsx*2);
            TInty = 1/(vInsx*2);
            precargaX = (int)(65536 - ((TIntx*250000)/1));
            precargaY = (int)(65536 - ((TInty*250000)/1));
            TMR1 = precargaX;
            TMR3 = precargaY;
            TMR1ON = 1;
            TMR3ON = 1;
        }
        
        if(counter>=20 && ac==1){
            ac=0;
            cru=1;
        }
        
        if(counterVC>=20&&cru ==1){//No alcanza a sumar a este counter a 21, pero si deberia hacer 21 veces el ciclo
            cru=0;
            counterVC = 0;
            des=1;
        }
        
        if(counter<=0 && des==1){
            if(pasosXDados>=pasosX && pasosYDados>=pasosY){
                pasosXDados=0;
                pasosYDados=0;
                counter=0;
                des=0;
                TMR1ON = 0;
                TMR3ON = 0;
                TMR0ON = 0;
            }
        }    
    }
}
