////prueba temp+mediamovil version 26/6/25
#include <main.h>
#include <can-PIC24.c>  

//SELECCION DE PINES DE CAN PARA CADA LINEA
///////////////Linea PRIVADA sensor /////////////////////////////////////
#pin_select C2TX = PIN_G0     //seleccionamos los piens de una linea de can _ CAN_B
#pin_select C2RX = PIN_A6        //coche principal

/////////////////Linea Principal ////////////////////////
#pin_select C1TX = PIN_D4  //SELECCIONAMOS LOS PINES DE LA SEGUNDA LINEA DE CAN CAN_A
#pin_select C1RX = PIN_D13    //

       

////////////////////// Definimos los valores de las id de can /////////////////
#define CAN_ID_SPEED_STEERING  0x180 
#define CAN_ID_SUSPEN_TEMP 0X181

#define CAN_BUFFER_LENGTH 8
#define CAN_PRIORITY 2 // Prioriza los mensajes del 1 al 3 (donde 1 es la máxima prioridad).


////////////////////////DEFINICION DE VARIABLES GLOBALES/////////////
//////////////////////////////////////////////////////////////////
unsigned int16 Steering_Sensor = 0, Zrack = 0, Caudal = 0, T1 = 0, T2 = 0, T3 = 0, T4 = 0;
unsigned int16 Suspen_Front_DRCH = 0, Suspen_Front_IZQ = 0, Auxiliar1 = 0,  Auxiliar2= 0;
volatile unsigned int8 Speed0 = 0, Speed1 = 0, Speed2 = 0, Speed3 = 0;

volatile char sendCanMsg = 0, readAdc = 0, TempSampCont=0;


const INT16 TIME_INTERVAL_10MS = 64000; //con 64000 ->setup_timer1(TMR_INTERNAL | TMR_DIV_BY_64 )


////////////////////// Definimos tamaño de los buffers que usaremos para mandar por de can1 /////////////////
unsigned int8 can_buffer[CAN_BUFFER_LENGTH];// recordatorio te da error si llamas buffer porque sera una funcion de alguna libreria
unsigned int8 can_buffer1[CAN_BUFFER_LENGTH];


#INT_TIMER1
void timer1_isr(VOID)
{
       
       sendCanMsg = 1; //poner variable a 1 mandar por CAN valores anteriores.
       readAdc = 1; //poner variable a 1 ADC guarda valores nuevos.
       set_timer1 (TIME_INTERVAL_10MS); 
       output_high(LED_1);
      
   clear_interrupt(int_timer1); ///////////////////////////////////////////////////////////////////////
   set_timer1 (TIME_INTERVAL_10MS);  // 

   
   can_enable_interrupts (RB) ;
   can2_enable_interrupts(RB);
   
   //VOLVEMOS A HABILITAR LAS INTERRUPCIONES POR SI ACASO, EN LAS DE CAN HACE FALTA   
   enable_interrupts (INT_CAN2); 
   enable_interrupts (INT_CAN1) ;
   enable_interrupts (INT_TIMER1);
   enable_interrupts (GLOBAL) ;
}

#INT_CAN1
void can1_isr (void)
   {
   //Cosas variadas que usa CAN
   
   struct rx_stat rxstat;                          //variable para la recepcion por can, la funcion pide este parametro
   int8 can_msg_size_0=8;
   volatile INT32 can_rcv_msg_id_2;
    volatile UNSIGNED int8 can_rcv_msg_2[8]={0};
   

      
   can_enable_interrupts (RB) ;
   can2_enable_interrupts(RB);
   
   //VOLVEMOS A HABILITAR LAS INTERRUPCIONES POR SI ACASO, EN LAS DE CAN HACE FALTA   
   enable_interrupts (INT_CAN2); 
   enable_interrupts (INT_CAN1) ;
   enable_interrupts (INT_TIMER1);
   enable_interrupts (GLOBAL) ;
   }
   

   #INT_CAN2
void can2_isr (void)
   {
   //Cosas variadas que usa CAN
   
   struct rx_stat rxstat;//variable para la recepcion por can, la funcion pide este parametro
   int8 can_msg_size_0=8;
   volatile INT32 can_rcv_msg_id_2;
   volatile UNSIGNED int8 can_rcv_msg_2[8]={0};
   
   can2_getd (can_rcv_msg_id_2, can_rcv_msg_2, can_msg_size_0, rxstat);
      
         Speed0=can_rcv_msg_2[2];
         Speed1=can_rcv_msg_2[3];
         Speed2=can_rcv_msg_2[4];
         Speed3=can_rcv_msg_2[5]; //cargar en la variable Speed lo que recibe para luego Speed mandarlo por CAN en un buffer  
         output_low(LED_1);
      
   can_enable_interrupts (RB) ;
   can2_enable_interrupts(RB);
   
   //VOLVEMOS A HABILITAR LAS INTERRUPCIONES POR SI ACASO, EN LAS DE CAN HACE FALTA   
   enable_interrupts (INT_CAN2); 
   enable_interrupts (INT_CAN1);
   enable_interrupts (INT_TIMER1);
   enable_interrupts (GLOBAL) ;
   }
   
   int8 readChanel(int ch) 
{
    set_adc_channel(ch);   // enable adc
    delay_us(10);          // wait time
    return read_adc();     // read ADC value (0-255)
}
unsigned int8 factorConversion(unsigned int16 MAX, unsigned int16 min, unsigned int16 data)
{
   unsigned int16 value = 0;
   
   value = ((data - min)*255)/(MAX - min);         //Cuidado con "int16", trunca los decimales
 
   if(data<=min){
   value=0;
   }
   if (value>=255){
   value=255;
   }
   
   return value;
}


unsigned int8 normalizacionSuspen(unsigned int16 lmed, unsigned int16 data){
   int16  desplazamiento = 0, valornormal=0;
   desplazamiento = data - lmed;
   
  // sacamos como un valor absoluto de lo que se desplaza
  
   if (desplazamiento < -150) { //aqui es que ha cruzado del 45 al 0)
    desplazamiento = desplazamiento + 255;// 45
    
      }
   else if (desplazamiento > 150) {//aqui hemos cruzado del 0 al 45 (0 a 255)
      desplazamiento = desplazamiento - 255;//-36
      }
   valornormal = 128 + desplazamiento;
   if (valornormal < 0) valornormal = 0;
   if (valornormal > 255) valornormal = 255;
   return valornormal;
 }
 

 

   VOID setup ()
   {



      //TAD=periodo del PIC y Tsampleo =31*TAD
      
      output_low(LED_1);
      set_tris_b (0b1100110000000000) ;
      set_tris_c (0b00001000);
      //set_tris_d (0b100000000000);
      //set_tris_d (0b00000000);
           ///// ADC interno del PIC//////
      setup_adc_ports(sAN9 | sAN10 | sAN11 | sAN14 | sAN15 | sAN24 | sAN25 | sAN26 | sAN27 | sAN28 | sAN29 | sAN30, VSS_VDD);
      setup_adc(ADC_CLOCK_INTERNAL | ADC_TAD_MUL_0);// AD1CON3=0x1F01 
      
      //Inicialización CAN
      can_init () ;
      can2_init () ;
      can_enable_b_transfer (TRB0) ; // De los buffers 0-7, CAN0 utiliza el TRB0 para el envío de datos.
      can2_enable_b_transfer (TRB1) ;
      
    
      can2_enable_b_receiver(RB9);// De los buffers 8-31, CAN2 utiliza el RB9 para recepción de datos.
      
      can2_enable_interrupts (RB); //ESTO IBA ASI - > can2_enable_interrupts (RB8); - > Y DABA PROBLEMAS, ESTARÍA BIEN MIRAR POR QUÉ Y ENTENDER EL DIAGRMAA DE CAN BIEN
      can_enable_interrupts (RB) ;
      
      can_set_mode(CAN_OP_CONFIG);   //ponemos modo config para poder cambiar el baud rate
      can_set_mode(CAN_OP_NORMAL);
      
      //Timer 1
      setup_timer1 (TMR_INTERNAL|TMR_DIV_BY_64) ;
      SET_TIMER1 (TIME_INTERVAL_10MS); // timer 0 overflow at 10 ms with 20 MHz
      
      setup_timer_2(TMR_INTERNAL | TMR_DIV_BY_64, 1562);
      set_timer2();
      
        
       can_enable_interrupts (RB) ;
       can2_enable_interrupts(RB);
      //Inerrupciones
      enable_interrupts (INT_TIMER1) ;
      enable_interrupts (INT_CAN2) ;
      enable_interrupts (INT_CAN1) ;
      enable_interrupts (GLOBAL) ;    

   }

   //////////////////////////////////////////////////////////////////////Hasta aqui el SETUP() !!!!!!!!!!!!!!!!!!!!
   /////////////////////////////////////////////////////////////////////////////
   /////////////////////////////////////////////////////////////////////////////
   ///////////////////////////MAIN
   /////////////////////////////////////////////////////////////////////////////
   /////////////////////////////////////////////////////////////////////////////
   VOID main () {
unsigned int8 i=0, Steering_SensorCalib = 0, ZrackCalib = 0, Suspen_Front_DRCHCalib = 0, Suspen_Front_IZQCalib = 0, CaudalCalib = 0, Temp1=0, Temp2=0, Temp3=0, Temp4=0, T1_filtr=0,T2_filtr=0,T3_filtr=0,T4_filtr=0;
unsigned int16 suma=0;

 setup () ;
   while(1){

    output_low(LED_1);
    output_high(LED_0);
    
       if (readAdc == 1)
       {
          readAdc= 0; //restart enable to read Adc channels    
         // leemos los pines analogicos (ej. t1 AN15) 
          T1 = readChanel(15); //DRCH tiene cinta
          T2 = readChanel(10);  //DRCH mas pegao al monocasco
          T3 = readChanel(14); //IZQ tiene cinta
          T4 = readChanel(11); //IZQ mas pegao al monocasco
 
          Steering_Sensor=readChanel(24);                                        // xq esta asi programado en el torque, como coloque al reves los cables del sensor y no me caté tenemos que hacer esto 
          Steering_SensorCalib=255-factorConversion(152,35,Steering_Sensor);// queremos que de izquierdas sea 0 y de derechas 255 
          
          Zrack=readChanel(25)>>2;
          ZrackCalib= factorConversion(255,0,Zrack); //ahora mismo este sensor no existe
          
          Suspen_Front_DRCH=readChanel(29);
          Suspen_Front_DRCHCalib= normalizacionSuspen(13,Suspen_Front_IZQ);
          
          Suspen_Front_IZQ=readChanel(30);
          Suspen_Front_IZQCalib = normalizacionSuspen(15,Suspen_Front_IZQ);
          
          Caudal=readChanel(30)>>2;
          CaudalCalib = factorConversion(255,0,Caudal);
          
       }
       if(sendCanMsg == 1)
       {
       
          //restart enable to send can message
          sendCanMsg = 0;
          
             
          can_buffer[0]=Speed0;
          can_buffer[1]=Speed1;
          can_buffer[2]=Speed2;
          can_buffer[3]=Speed3;
          can_buffer[4]=Steering_Sensor;
          can_buffer[5]=Steering_SensorCalib; // Izq FA - Drch 07 - centro 80 (HEX)
          can_buffer[6]=Zrack;
          can_buffer[7]=ZrackCalib;
            
          can_buffer1[0]=Suspen_Front_DRCH; 
          can_buffer1[1]=Suspen_Front_DRCHCalib; 
          can_buffer1[2]= Suspen_Front_IZQ;
          can_buffer1[3]=Suspen_Front_IZQCalib; //CaudalCalib;desplazamiento
          can_buffer1[4]=T1;
          can_buffer1[5]=T2;
          can_buffer1[6]=T3;
          can_buffer1[7]=T4;
            
          //send data via CAN
          can_putd(CAN_ID_SPEED_STEERING, can_buffer, CAN_BUFFER_LENGTH, CAN_PRIORITY, false, false); //ID 180 HEX / 384 DEC
          delay_ms(1);
          can_putd(CAN_ID_SUSPEN_TEMP, can_buffer1, CAN_BUFFER_LENGTH, CAN_PRIORITY, false, false); //ID 181 HEX / 385 DEC
          delay_ms(2);            
           //change led status to know if pic is sending by CAN 0
       }
   }
   
  
}


