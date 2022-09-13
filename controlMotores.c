/*
 * File:   controlMotores.c
 * Author: ivanm
 *
 * Created on 10 de septiembre de 2022, 12:26 PM
 */
#include <xc.h>
#pragma config FOSC=INTOSC_EC
#define _XTAL_FREQ 8000000
#pragma config WDT = OFF
#pragma config PBADEN = OFF
#pragma config LVP=OFF

#define F_X RB0
#define F_Y RB1
#define F_Z RB2
#define dir_X LATD2
#define dir_Y LATD3
#define dir_Z LATD4
#define step_X LATD0
#define step_Y LATD1
#define step_Z LATD5

void __interrupt() ISR(void);
void precargas(void);
void goHome(void);
void mover(long int x,long int y,int absoluto);
void moverZ(long int z,int absoluto);
void Transmitir(unsigned char dato);
void guardarV(void);
void sacarV(void);
const int tDelay = 400;
const int tDelayZ = 1000;
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
int finX,finY,finZ,movflag;             //Banderas de finalización
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
                                {8700,19000}};              //StandBy - Posiciones absolutas
unsigned int posX,posY,posZ,carro,vactual,vs;        //Posiciones del robot
unsigned char orden,estado,ope;         //Comunicación

void main(void) {
    
    Ta = 4.5; //s
    deltaT = Ta/20;//s

      ADCON1=0b00001111;  
    //Configuramos tres pines del puerto D como salida para los pulsos de los tres motores
    TRISD = 0b11000000; //D0 xStep, D1 YStep, D2 xDir, D3 yDir, D4 zDir, D5 ZStep
    LATD = 0;
    RBPU=0;
    //Direcciones
    //LATD2 = 1;      //Dirección motor X
    //LATD3 = 1;      //Dirección motor Y
    //LATD4 = 1;      //Dirección motor Z
    
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
    TMR0ON = 0;
    TMR0 =9286;        //Interrupciones cada 0.225 s
    
    //Configuración TMR1
    T1CON = 0b10100000;
    TMR1ON = 0;
    
    //Configuración TMR3
    T3CON = 0b10100000;
    TMR3ON = 0;
    
    //configuración comunicación serial
    RCSTA=0b10010000;
    TXSTA=0b00100000;
    BAUDCON=0b00000000;
    SPBRG=12;
    
    //interrupcion recepción de datos.
    RCIP=1;
    RCIF=0;
    RCIE=1;
    //Habilitamos las interrupciones perifericas
    PEIE = 1;
    
    //Habilitamos todas las interrupciones
    GIE = 1;
    
    //hacemos set al origen
    goHome();
    finX=1;
    finY=1;
    finZ=1;
    movflag=1;
    __delay_ms(500);
    Transmitir('R');
    Transmitir('R');
    moverZ(1500,0);
    moverZ(-1500,0);
    mover(posiciones[10][0],posiciones[10][1],1);
    Transmitir('R');
    while(1){
        
       // if(finX==1 && finY==1 && finZ==1){
         //   movflag=1;
       // }else{
        //    movflag=0;
      //  }
        
        if(ope=='H'){
            Transmitir('o');
            if(vactual!=0)guardarV();
            goHome();
             ope='0';
            Transmitir('e');
        }
        if(ope=='G'){
            Transmitir('o');
            guardarV();
            ope='0';
            Transmitir('e');
        }
        if(ope=='S'){
            Transmitir('o');
            //mover(1000,1000,15000,0);
            sacarV();
            ope='0';
            Transmitir('e');
        }
    }
}

void __interrupt() ISR(void){
    if(RCIF==1){
        orden=RCREG;
        Transmitir(orden);
            if(orden=='1')carro=1;
            if(orden=='2')carro=2;
            if(orden=='3')carro=3;
            if(orden=='4')carro=4;
            if(orden=='5')carro=5;
            if(orden=='6')carro=6;
            if(orden=='7')carro=7;
            if(orden=='8')carro=8;
            if(orden=='9')carro=9;
            if(orden=='G'||orden=='g')ope='G';
            if(orden=='S'||orden=='s')ope='S';
            if(orden=='H'||orden=='h')ope='H';
       RCIF=0;
    }
    
    if(TMR0IF==1){
        TMR0IF=0;
        TMR0 = 9286;
        
        if (ac==1 && counter==-1){
            if(pasosX>0){
                TMR1ON=1;       //Encender el Timer 1
            }
        
            if(pasosY>0){
                TMR3ON=1;       //Encender el Timer 3
            }
        }
        
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
            Transmitir('W');
            TMR0ON = 0;
            TMR1ON = 0;
            TMR3ON = 0;
            finY=0;
            finX=0;
            movflag=1;
        }
    }
    
    if(TMR1IF==1){
        TMR1IF=0;
        TMR1 = precargaX;
        
        if(step_X==1 && pasosXDados<pasosX){
            pasosXDados++;
            step_X=0;
        } else if(step_X==0 && pasosXDados<pasosX){
            step_X=1;
        }  else if(pasosXDados>=pasosX){
            finX = 1;
            TMR1ON = 0;
            pasosXDados=0;
        }
        
    }
    
    if(TMR3IF==1){
        TMR3IF=0;
        TMR3 = precargaY;
        
        if(step_Y==1 && pasosYDados<pasosY){
            pasosYDados++;
            step_Y = 0;
        } else if(step_Y==0 && pasosYDados<pasosY){
            step_Y = 1;
        } else if(pasosYDados>=pasosY){
            finY = 1;
            TMR3ON = 0;
            pasosYDados=0;
        }
    }
}
void Transmitir(unsigned char dato){
    //se configura momentaneamente para envio
    while(TRMT==0);
    TXREG=dato;
}
void precargas(void){
   int i;
   for (i=1;i<21;i++){
       
        vInsx = i*v5x;
        vInsy = i*v5y;
        
        if(vInsx>0){
            TIntx = 0.5/vInsx;//1.0/(vInsx*2.0);
            arrayX[i-1] = (unsigned int)(65536 - ((TIntx*2000000)/4));
        }
        
        if(vInsy>0){
            TInty = 0.5/vInsy;
            arrayY[i-1] = (unsigned int)(65536 - ((TInty*2000000)/4));
        }        
   }
}

void goHome(void){
    //Direcciones
    dir_X = 0;      //Dirección motor X
    dir_Y = 0;      //Dirección motor Y
    dir_Z = 0;      //Dirección motor Z
    
    
    while(F_Z==1){  //Devolver el Z
        step_Z=1;
        __delay_us(tDelayZ);
        step_Z=0;
        __delay_us(tDelayZ);
    }
    
    while(F_X==1 || F_Y==1){
        if(F_X==1){
            step_X=1;    //Step X
        }
        
        if(F_Y==1){
            step_Y=1;    //Step Y
        }
        
        __delay_us(tDelay);
        
        step_X=0;
        step_Y=0;
        
        __delay_us(tDelay);
    }
    
    posX=0;
    posY=0;
    posZ=0;
}



void guardarV(){
    if(vactual!=0){

        if(vactual==1){
            mover(posiciones[2][1],posiciones[2][2],1);
        }
        else if(vactual==2){
            Transmitir('s');
            Transmitir('2');
            mover(posiciones[3][1],posiciones[3][2],1);
        }
        else if(vactual==3){
            mover(posiciones[4][1],posiciones[4][2],1);
        }
        else if(vactual==4){
            mover(posiciones[5][1],posiciones[5][2],1);
        }
        else if(vactual==5){
            mover(posiciones[6][1],posiciones[6][2],1);
        }else if(vactual==6){
            mover(posiciones[7][1],posiciones[7][2],1);
        }else if(vactual==7){
            mover(posiciones[8][1],posiciones[8][2],1);
        }else if(vactual==8){
            mover(posiciones[9][1],posiciones[9][2],1);
        }else if(vactual==9){
            mover(posiciones[10][1],posiciones[10][2],1);
        }
        moverZ(1500,0);
        mover(0,-400,0);
        moverZ(-1500,0);
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
    estado='e';
  }

void sacarV(){
    vs=carro;
    if(vactual==0){        
        if(vs==1){
            mover(posiciones[1][0],posiciones[1][1],1);//mover(posiciones[2][0],posiciones[2][1],800,1);
            vactual=1;
        }
        else if(vs==2){
            Transmitir('s');
            Transmitir('2');
            mover(posiciones[2][0],posiciones[2][1],1);
            vactual=2;
        }
        else if(vs==3){
            mover(posiciones[3][0],posiciones[3][1],1);
            vactual=3;
        }
        else if(vs==4){
            mover(posiciones[4][0],posiciones[4][1],1);
            vactual=4;
        }
        else if(vs==5){
            mover(posiciones[5][0],posiciones[5][1],1);
            vactual=5;
        }else if(vs==6){
            mover(posiciones[6][0],posiciones[6][1],1);
            vactual=6;
        }else if(vs==7){
            mover(posiciones[7][0],posiciones[7][1],1);
            vactual=7;
        }else if(vs==8){
            mover(posiciones[8][0],posiciones[8][1],1);
            vactual=8;
        }else if(vs==9){
            mover(posiciones[9][0],posiciones[9][1],1);
            vactual=9;
        }
        Transmitir('1');
        mover(0,-400,0);
        Transmitir('2');
        moverZ(1500,0);
        Transmitir('3');
        mover(0,400,0);
        Transmitir('4');
        moverZ(-1500,0);
        Transmitir('5');
        mover(posiciones[10][0],posiciones[10][1],1);
        vs=0;
      }
    else{
        Transmitir('m');
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
    estado='e';
}
void mover(long int x,long int y,int absoluto){
    movflag=0;
    
  //  if(movflag==1){
        Transmitir('M');
        if(absoluto==1){    //Absoluto

            if(x>posX){
                pasosX = (unsigned int)(x-posX);
                dir_X = 1;      //Dirección motor X
            }else{
                pasosX = (unsigned int)(posX-x);
                dir_X = 0;      //Dirección motor X
            }

            if(y>posY){
                pasosY = (unsigned int)(y-posY);
                dir_Y = 1;      //Dirección motor y
            }else{
                pasosY = (unsigned int)(posY-y);
                dir_Y = 0;      //Dirección motor y
            }

            posX=x;
            posY=y;
        }else{      //Incremental
            if(x>0){
                pasosX = (unsigned int)(x);
                dir_X = 1;      //Dirección motor X
            }else{
                pasosX = (unsigned int)(-x);
                dir_X = 0;      //Dirección motor X
            }

            if(y>0){
                pasosY = (unsigned int)(y);
                dir_Y = 1;      //Dirección motor y
            }else{
                pasosY = (unsigned int)(-y);
                dir_Y = 0;      //Dirección motor y
            }

            posX=posX+x;
            posY=posY+y;
        }

        if (pasosX>0){
            pasosXDados = 0;
            vCx = (pasosX*0.5)/Ta;
            v5x = 0.05*vCx;
            finX=0;
        } else{
            finX=1;
        }
    
        if (pasosY>0){
            pasosYDados = 0;
            vCy = (pasosY*0.5)/Ta;
            v5y = 0.05*vCy;
            finY=0;
        } else{
            finY=1;
        }
    
        if(pasosX>0 || pasosY>0){
        
            counter = -1;
            counterVC = 0;
            ac=1;
            des=0;
            cru=0;
        
            precargas();
            TMR0ON=1;           //Encendemos el TMR0 para iniciar el proceso
            TMR0IF=0;
            TMR0 = 9286;
        
        }
   // }

        while(!movflag){
            
            if(movflag){
             Transmitir('E');
                break;
            }

        }
    //funcion para mover Z

}
void moverZ(long int z, int absoluto){
    

        Transmitir('M');
        if(absoluto==1){    //Absoluto
            if(z>posZ){
                pasosZ = (unsigned int)(z-posZ);
                dir_Z = 1;      //Dirección motor Z
            }else{
                pasosZ = (unsigned int)(posZ-z);
                dir_Z = 0;      //Dirección motor Z
            }
            posZ=z;
        }else{      //Incremental
            
            if(z>0){
                pasosZ = (unsigned int)(z);
                dir_Z = 1;      //Dirección motor Z
            }else{
                pasosZ = (unsigned int)(-z);
                dir_Z = 0;      //Dirección motor Z
            }
            posZ=posZ+z;
        }

    //funcion para mover Z
   for(int i=0;i<pasosZ;i++){
      step_Z = 1;
      __delay_us(tDelayZ);
      step_Z = 0;
      __delay_us(tDelayZ);
      }
        pasosZ=0;
}

