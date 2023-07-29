// DMATimer.c
// Runs on LM4F120
// Periodic timer triggered DMA transfer
// Uses Timer5A to trigger the DMA, read from an 8-bit PORT, and then write to a memory Buffer
// There is a Timer5A interrupt after the buffer is full
// Jonathan Valvano
// January 1, 2013

/* This example accompanies the book
   "Embedded Systems: Real Time Operating Systems for ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2012
   Section 6.4.5, Program 6.1

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#include "tm4c1294ncpdt.h"

// The control table used by the uDMA controller.  This table must be aligned to a 1024 byte boundary.
uint32_t ucControlTable[256] __attribute__ ((aligned(1024)));
// Timer5A uses uDMA channel 8 encoding 3
#define CH8 (8*4)
#define CH8ALT (8*4+128)
#define CH2 (2*4)
#define CH2ALT (2*4+128)
#define BIT8 0x00000100
#define BIT2 0x00000004

unsigned long CounTimer=0;
// ***************** Timer5A_Init ****************
// Activate Timer5A trigger DMA periodically
// Inputs:  period in usec
// Outputs: none
//void Timer5A_Init(unsigned short period){ volatile unsigned long Delay;
void Timer5A_Init(void){ volatile unsigned long Delay;
  SYSCTL_RCGCTIMER_R |= 0x20;      // 0) activate timer5
  while((SYSCTL_PRTIMER_R & 0x20)==0);
  Delay = 0;                       // wait for completion
  TIMER5_CTL_R &= ~0x00000001;     // 1) disable timer5A during setup
  TIMER5_CFG_R = 0x00000004;       // 2) configure for 16-bit timer mode
  TIMER5_TAMR_R = 0x00000002;      // 3) configure for periodic mode, default down-count settings
  TIMER5_TAILR_R = 0x00FFFF;       // 4) reload value
  //TIMER5_TAILR_R = period-1;       // 4) reload value
  TIMER5_TAPR_R = 0xA0;              // 5) 1us timer5A
  //TIMER5_TAPR_R = 49;              // 5) 1us timer5A
  TIMER5_ICR_R = 0x00000001;       // 6) clear timer5A timeout flag
  TIMER5_IMR_R |= 0x00000001;      // 7) arm timeout interrupt
  NVIC_PRI23_R = (NVIC_PRI23_R&0xFFFFFF00)|0x00000040; // 8) priority 2


}
// ***************Timer3A_Init*********************
// Configura y activa el timer 3A para una solicitud
// periodica de transferencias por DMA
// entradas: Periodo de cuenta en us
void Timer3A_Init(uint16_t period){
    uint32_t ui32Loop;
        SYSCTL_RCGCTIMER_R |= 0x08;           //0) Activar Timer_3
        ui32Loop = SYSCTL_RCGCGPIO_R;         //retardo de activacion

/*********** Afecta a los timers A y B *****************/
        TIMER3_CTL_R = 0X00000000;       //1) Deshabilita el timer durante configuracion
        TIMER3_CFG_R = 0X00000004;       //2) Configura modo de 16-bit
/*********** Afecta al timer A *************************/
        TIMER3_TAMR_R = 0x00000002;      //3) Configura modo periodico (recarga automatica), y modo de cuenta hacia abajo
        TIMER3_TAILR_R = 65*(period-1);  //4) Valor de recarga de 16 bits
        TIMER3_TAPR_R  =       0xFF;     //5) Valor del prescalador-1 para 1ms timer_3A
        TIMER3_ICR_R   = 0x00000001;     //6) Limpia banderas pendientes del fin de cuenta
        TIMER3_IMR_R |= 0x00000001;      //7) Desenmascara interrupcion del timer_3A

        //Puerto N
        SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R12;     //1) Reloj
        GPIO_PORTN_DIR_R = 0x01;                     //2) Direccion de salida
        GPIO_PORTN_DEN_R = 0x01;                     //3) Modo digital
        GPIO_PORTN_DATA_R = 0x00;                    //4) Dato de salida = 0
}

void Puertos_UART(void){
    SYSCTL_RCGCGPIO_R |= 0x01;          //0) Reloj Port_A
    GPIO_PORTA_AFSEL_R |= 0x03;         //1) Habilita Funcion Alterna - Pines A0 y A1
    GPIO_PORTA_DEN_R |= 0x03;           //3) Modo digital - Pines A0 y A1
    GPIO_PORTA_PCTL_R |= 0x00000011;    //4) Funcion Alterna para A0/A1 = UART
    GPIO_PORTA_AMSEL_R &= ~0x03;        //5) Deshabilita modo analogico de Pines A0 y A1
}
void UART0_init(void){
    SYSCTL_RCGCUART_R |=  0x00000001;           //0) Reloj UART_0 (p.505)
    while((SYSCTL_PRUART_R&0x01) == 0){};       //retardo de activacion
    UART0_CTL_R &= ~0x00000001;                 //1) Deshabilita el UART durante configuracion
                                                //2) Configura baud rate = 9600 bauds
    UART0_IBRD_R = 104;                         // Valor entero       IBRD = int(16,000,000 / (16 * 9,600)) = int(104.16666)
    UART0_FBRD_R = 11;                          // Valor fraccionario FBRD = round(0.1667 * 64) = 11
    UART0_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);            //3) Configura tamaño de palabra = 8 bits y habilita el modo FIFO
    UART0_CC_R = (UART0_CC_R&~UART_CC_CS_M)+UART_CC_CS_PIOSC;   //4)  Fuente alterna de reloj ALTCLKCFG para el UART
    SYSCTL_ALTCLKCFG_R = (SYSCTL_ALTCLKCFG_R&~SYSCTL_ALTCLKCFG_ALTCLK_M)+SYSCTL_ALTCLKCFG_ALTCLK_PIOSC;
    //5) Fuente de reloj alterna = PIOSC
    UART0_CTL_R &= ~0x00000020;                 //6) Divisor de frecuencia = x16 para UART
    //UART0_IFLS_R &= 0xF8;                       //7) Nivel de FIFO = 1/8 de capacidad = 16 bits = Rafagas de 2 bytes
    UART0_CTL_R |= 0x00000001;                  //8) Habilita UART_0
}


// ************DMA_Init*****************
// Inicializa en DMA para realizar transferenicas solicitadas por el timer3A
// Se ejecuta previo a cada solicitud de una transferencia completa

void DMA_Init(void){
  volatile unsigned long delay;
  //int i;
//  for(i=0; i<256; i++){
//    ucControlTable[i] = 0;
//  }
  SYSCTL_RCGCDMA_R |= 0x01;                                 //0) Reloj uDMA 1
  delay = SYSCTL_RCGCDMA_R;                                 //retardo de activacion
  UDMA_CFG_R = 0x01;                                        //1) Habilitar operacion de DMA
  UDMA_CTLBASE_R = (uint32_t)ucControlTable;                //2) Ubicar la tabla de control para los canales
  UDMA_CHMAP0_R = (UDMA_CHMAP0_R&0xFFFFF0FF)|0x00000100;    //3) Fuente de solicitud. Canal 2 = timer3A
  UDMA_PRIOCLR_R = 0xFF;     //4) No hay prioridades
  UDMA_ALTCLR_R = BIT2;      //5) Canal 2 usa control primario
  UDMA_USEBURSTSET_R = BIT2; //6) Canal  2 acepta solicitudes de rafagas*timer=brst
  UDMA_REQMASKCLR_R = BIT2;  //7) Deshabilita al uDMA a atender solicitudes del canal 2

}

uint16_t *SourcePt;                 //Direccion fija del origen
volatile uint32_t *DestinationPt;   //Direccion fija del destino
uint32_t Count_Word;                //Cantidad de transferencias segun el tamaño de palabra

//Funcion privada para reprogramar la parte principal de un canal de la estructura de control

void Set_Regular(void){
  ucControlTable[CH2]   = (uint32_t)SourcePt;               //0) Direccion origen
  ucControlTable[CH2+1] = (uint32_t)DestinationPt;          //1) Direccion destino
  ucControlTable[CH2+2] = 0xC0008001+((Count_Word-1)<<4);   //2) DMA Channel Control Word (DMACHCTL)
/* DMACHCTL          Bits    Valor Descripcion
   DSTINC            31:30   11    no incremento en la direccion destino
   DSTSIZE           29:28   00    Tamano de dato = 8-bit en destino
   SRCINC            27:26   00    incremento en la direccion origen = 8 bit
   SRCSIZE           25:24   00    Tamano de dato = 8-bit en origen
   reserved          23:18   0     Reservado
   ARBSIZE           17:14   0011     Arbitrataje cada 4 transferencias
   XFERSIZE          13:4  countWord-1 Total de transferencias
   NXTUSEBURST       3       0      no aplica para modo basico
   XFERMODE          2:0     01     Modo basico
  */

}

// ************DMA_Star*****************
// configura las direcciones de origen y destino de las transferencias solicitadas
// transferencias de 8 bits en rafagas de 4 bytes
// La direccion del origen se incrementa en 8 bits
// Entradas: source Apuntador de un buffer en RAM que contiene valores de 0 a 255
//          destination Registro FIFO de transmision UART0Tx
//          count numero de bytes a transferir (max 1024 palabras)
// Esta rutina no espera la finalizacion de la transferencia
//Se deben haber habilitado todos los modulos antes (GPIO, UART, uDMA y Timer)
void DMA_Star(uint8_t *source, volatile uint32_t *destination, uint32_t count){
    SourcePt = source+count-1;
    DestinationPt = destination;
    Count_Word = count;
    Set_Regular();              //0) Configura los parametros de la transmision

    TIMER3_DMAEV_R |= 0x01;      //1) Activa dma_req signal para evento time-out pp1019

    NVIC_EN1_R = 1<<(35-32);     //2) Habilita interrupcion 35 para timer_3A
    TIMER3_CTL_R |= 0x00000001;  //3) Habilita timer_3

    UDMA_ENASET_R |= BIT2;       //4) Habilita el canal 2 a hacer solicitudes de transferencia

    // bit 8 en UDMA_ENASET_R se limpia cuando termina
    // bits 2:0 ucControlTable[CH8+2] se limpia cuando terminan las transferenicas
}


// ************DMA_Status*****************
// Checa cuantas transferencias se han realizado
// Entradas: nada
// Salidas: contador de transferencias completadas
uint32_t DMA_Status(void){
  return CounTimer;     //contador de transferencias completadas
  // µDMA Channel 8 enable bit is high if active
}

//*************DMA_Stop********************
//Inhabilita al DMA a recibir mas solicitudes de transferencia
void DMA_Stop(){
    UDMA_ENACLR_R = BIT2;       //0) Deshabilitar canal 2 a realizar solicitudes
    NVIC_DIS1_R = 0x00000008;   //1) Desactiva la interrupcion directo del NVIC
    TIMER3_CTL_R &=~0x01;       //2) Desactiva el timmer para evitar mas interrupciones dado el modo periodico
}





void Timer3A_Handler(void){
        CounTimer++;                //0) Transferencias completas
        if (CounTimer>64){
            TIMER3_CTL_R &= 0X0;    //1) Desactiva el timmer para evitar mas interrupciones dado el modo periodico
        }
        TIMER3_ICR_R = 0x00000001; //2) Limpia la bandera de interrupcion fin de cuenta

        if((UDMA_ENASET_R & BIT2)==0){
            Set_Regular();          //3) Reconfigurar los parametros de transferencia
        }

        //GPIO_PORTN_DATA_R ^= 0x01;

        //        if((ucControlTable[CH8+2]&0x0007)==0){
        //            Set_Regular();}
        //
        //        if((ucControlTable[CH8ALT+2]&0x0007)==0){
        //              Set_Alternate();}
}



//void Set_Regular(void){
//  ucControlTable[CH2]   = (uint32_t)SourcePt;                 // first and last address
//  ucControlTable[CH2+1] = (uint32_t)DestinationPt;    // last address
//  ucControlTable[CH2+2] = 0xD5000001+((Count_Word-1)<<4);             // DMA Channel Control Word (DMACHCTL)
///* DMACHCTL          Bits    Value Description
//   DSTINC            31:30   11    no destination address increment
//   DSTSIZE           29:28   01    16-bit destination data size
//   SRCINC            27:26   01     16-bit source address increment
//   SRCSIZE           25:24   01     16-bit source data size
//   reserved          23:18   0     Reserved
//   ARBSIZE           17:14   0     Arbitrates after 1 transfer
//   XFERSIZE          13:4  countWord-1 Transfer count items
//   NXTUSEBURST       3       0      N/A for this transfer type
//   XFERMODE          2:0     01      Use basic transfer mode
//  */
//
//}


//void DMA_Star(uint16_t *source, volatile uint32_t *destination, uint32_t count){
//    SourcePt = source+count-1;
//    DestinationPt = destination;
//    Count_Word = count;
//    Set_Regular();
//    //Set_Alternate();
//    TIMER3_DMAEV_R |= 0x01;         // Activa dma_req signal para evento time-out pp1019
//
//    NVIC_EN1_R = 1<<(35-32);     // HABILITA LA INTERRUPCION DE  TIMER3
//    TIMER3_CTL_R |= 0x00000001;  // HABILITA TIMER A
//    //NVIC_EN2_R |= 0x10000000;        // 9) enable interrupt 19 in NVIC
//      // vector number 108, interrupt number 92
//
//    //TIMER5_CTL_R |= 0x00000001;      // 10) enable timer5A
//   // interrupts enabled in the main program after all devices initialized
//
//    UDMA_ENASET_R |= BIT2;  // µDMA Channel 2 is enabled.
//    // bit 8 in UDMA_ENASET_R become clear when done
//    // bits 2:0 ucControlTable[CH8+2] become clear when done
//}

//Funcion privada usada pa reprogramar la parte alterna de un canal de la estructura de control
//void Set_Alternate(void){
//    ucControlTable[CH2ALT] = (uint32_t)SourcePt;        //la misma que el regular
//    ucControlTable[CH2ALT+1] = (uint32_t)DestinationPt;   //ultima direccion
//    ucControlTable[CH2ALT+2] = 0xD5000001+((Count_Word-1)<<4);//DMA Control de palabra de canal
//}
//Set_Alternate();
