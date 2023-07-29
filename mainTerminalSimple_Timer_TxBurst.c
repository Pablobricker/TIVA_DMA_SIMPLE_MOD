
#include <DMA_UART_TIMER_TXBURST.h>
#include <stdint.h>
#include "tm4c1294ncpdt.h"
//#include "SPI_POT_DRIVERS.h"
//#include "VL53_I2C_dirvers.h"

//DMA_Init(void)
//TRANSFERENCIAS DE SALIDA DMA POR RAFAGAS DE UART 32 BITS CADA RAFAGA 4 TRANSMISIONES DE UART POR RAFAGA
//256 DATOS PARA SACAR LA TABLA DE RAMPA, 64 RAFAGAS DE 32 BITS
//PARA TRANSFERENCIAS SIMPLES CAMBIAR LA EL TAMA�O DE TRANSFERTENCIA Y EL TMA�O DE SALTO DE DIRECCION EN SETREGULAR()
//Y MOVER LA CONDICION DE FIN DE CONTEO EN LA RUTINA DE INTERRUPCION DEL CONTADOR TIMER3A_HANDLER()
//Y EN LA INICIALIZACION DEL DMA ESCRIBIR EL REG USEBURST_CLR
//NO MOVER LA FIFO DE UART, PORQUE DE TODAS FORMAS NO TOMA DE AHI EL CANAL PARA HJACER LAS TRANSFERENICAS
//PARA CAMBIAR LA RAPIDEZ CAMBIA EL VALOR DE CARGA DEL TIMMER TAIL_R

//buffer para transferencia por spi PENDIENTE
//condiciones del spi tx
//uint16_t seno_slider[] = {0x1180, 0x1183, 0x1186, 0x1189, 0x118c, 0x118f, 0x1192, 0x1195, 0x1198, 0x119c, 0x119f, 0x11a2, 0x11a5, 0x11a8, 0x11ab, 0x11ae,
//0x11b0, 0x11b3, 0x11b6, 0x11b9, 0x11bc, 0x11bf, 0x11c1, 0x11c4, 0x11c7, 0x11c9, 0x11cc, 0x11ce, 0x11d1, 0x11d3, 0x11d5, 0x11d8,
//0x11da, 0x11dc, 0x11de, 0x11e0, 0x11e2, 0x11e4, 0x11e6, 0x11e8, 0x11ea, 0x11eb, 0x11ed, 0x11ef, 0x11f0, 0x11f2, 0x11f3, 0x11f4,
//0x11f6, 0x11f7, 0x11f8, 0x11f9, 0x11fa, 0x11fb, 0x11fb, 0x11fc, 0x11fd, 0x11fd, 0x11fe, 0x11fe, 0x11fe, 0x11ff, 0x11ff, 0x11ff,
//0x11ff, 0x11ff, 0x11ff, 0x11ff, 0x11fe, 0x11fe, 0x11fd, 0x11fd, 0x11fc, 0x11fc, 0x11fb, 0x11fa, 0x11f9, 0x11f8, 0x11f7, 0x11f6,
//0x11f5, 0x11f4, 0x11f2, 0x11f1, 0x11ef, 0x11ee, 0x11ec, 0x11eb, 0x11e9, 0x11e7, 0x11e5, 0x11e3, 0x11e1, 0x11df, 0x11dd, 0x11db,
//0x11d9, 0x11d7, 0x11d4, 0x11d2, 0x11cf, 0x11cd, 0x11ca, 0x11c8, 0x11c5, 0x11c3, 0x11c0, 0x11bd, 0x11ba, 0x11b8, 0x11b5, 0x11b2,
//0x11af, 0x11ac, 0x11a9, 0x11a6, 0x11a3, 0x11a0, 0x119d, 0x119a, 0x1197, 0x1194, 0x1191, 0x118e, 0x118a, 0x1187, 0x1184, 0x1181,
//0x117e, 0x117b, 0x1178, 0x1175, 0x1171, 0x116e, 0x116b, 0x1168,
//    0x1165, 0x1162, 0x115f, 0x115c, 0x1159, 0x1156, 0x1153, 0x1150,
//    0x114d, 0x114a, 0x1147, 0x1145, 0x1142, 0x113f, 0x113c, 0x113a,
//    0x1137, 0x1135, 0x1132, 0x1130, 0x112d, 0x112b, 0x1128, 0x1126,
//    0x1124, 0x1122, 0x1120, 0x111e, 0x111c, 0x111a, 0x1118, 0x1116,
//    0x1114, 0x1113, 0x1111, 0x1110, 0x110e, 0x110d, 0x110b, 0x110a,
//    0x1109, 0x1108, 0x1107, 0x1106, 0x1105, 0x1104, 0x1103, 0x1103,
//    0x1102, 0x1102, 0x1101, 0x1101, 0x1100, 0x1100, 0x1100, 0x1100,
//    0x1100, 0x1100, 0x1100, 0x1101, 0x1101, 0x1101, 0x1102, 0x1102,
//    0x1103, 0x1104, 0x1104, 0x1105, 0x1106, 0x1107, 0x1108, 0x1109,
//    0x110b, 0x110c, 0x110d, 0x110f, 0x1110, 0x1112, 0x1114, 0x1115,
//    0x1117, 0x1119, 0x111b, 0x111d, 0x111f, 0x1121, 0x1123, 0x1125,
//    0x1127, 0x112a, 0x112c, 0x112e, 0x1131, 0x1133, 0x1136, 0x1138,
//    0x113b, 0x113e, 0x1140, 0x1143, 0x1146, 0x1149, 0x114c, 0x114f,
//    0x1151, 0x1154, 0x1157, 0x115a, 0x115d, 0x1160, 0x1163, 0x1167,
//    0x116a, 0x116d, 0x1170, 0x1173, 0x1176, 0x1179, 0x117c, 0x1180
//};

//uint16_t seno_slider[256] = {  // must be in RAM, can't DMA out of ROM
//  2048,2097,2146,2195,2244,2293,2341,2390,2438,2486,2534,2581,2629,2675,2722,2768,
//  2813,2858,2903,2947,2991,3034,3076,3118,3159,3200,3239,3278,3317,3354,3391,3427,
//  3462,3496,3530,3562,3594,3625,3654,3683,3711,3738,3763,3788,3812,3834,3856,3876,
//  3896,3914,3931,3947,3962,3976,3988,3999,4010,4019,4026,4033,4038,4043,4046,4047,
//  4048,4047,4046,4043,4038,4033,4026,4019,4010,3999,3988,3976,3962,3947,3931,3914,
//  3896,3876,3856,3834,3812,3788,3763,3738,3711,3683,3654,3625,3594,3562,3530,3496,
//  3462,3427,3391,3354,3317,3278,3239,3200,3159,3118,3076,3034,2991,2947,2903,2858,
//  2813,2768,2722,2675,2629,2581,2534,2486,2438,2390,2341,2293,2244,2195,2146,2097,
//  2048,1999,1950,1901,1852,1803,1755,1706,1658,1610,1562,1515,1467,1421,1374,1328,
//  1283,1238,1193,1149,1105,1062,1020,978,937,896,857,818,779,742,705,669,634,600,
//  566,534,502,471,442,413,385,358,333,308,284,262,240,220,200,182,165,149,134,120,
//  108,97,86,77,70,63,58,53,50,49,48,49,50,53,58,63,70,77,86,97,108,120,134,149,165,
//  182,200,220,240,262,284,308,333,358,385,413,442,471,502,534,566,600,634,669,705,
//  742,779,818,857,896,937,978,1020,1062,1105,1149,1193,1238,1283,1328,1374,1421,
//  1467,1515,1562,1610,1658,1706,1755,1803,1852,1901,1950,1999};


uint8_t seno_slider[256] = {
                        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
                        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
                        0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
                        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
                        0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F,
                        0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F,
                        0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F,
                        0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F,
                        0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F,
                        0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F,
                        0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF,
                        0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF,
                        0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF,
                        0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF,
                        0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE, 0xEF,
                        0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF

};
void main(void){
    //Puertos_SPI();
    //SSI0_init();
    //I2C_Init();
    //VL53_Init();
    Puertos_UART();     //Configura puertos
    UART0_init();       //Configura uart
    DMA_Init();         //Configura dma
    Timer3A_Init(1000);     //Configura timer ms

    DMA_Star(seno_slider,0x4000C000,256);  //Configura estructura de control de canales
    while(1);

}

