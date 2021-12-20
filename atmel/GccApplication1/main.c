/*
 * GccApplication1.c
 *
 * Created: 2019-08-28 오후 12:13:16
 * Author : CDSL
 */ 

#include "mcu_init.h"
#include "dataType.h"

// 이게 진짜 프로젝트 코드임
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define Ke 0.0683
#define Kt 0.0683

volatile int32_t g_Cnt, g_preCnt;

/////////////////////////////////////////////////
////////////////     Variables    ///////////////
/////////////////////////////////////////////////
// 위치 변수
volatile double g_Pdes = 0., g_Ppre; // desired value, 이전의 position error
volatile double g_Pcur, g_Pvcur;    // 현재 position
volatile double g_Perr, g_Pperr = 0; // 현재의 position error

// 속도 변수
volatile double g_Vcur;            // Angular Velocity
volatile double g_Vdes = 0.2;      // desired Angular Velocity
volatile double g_Verr;            // 현재의 velocity error
volatile double V_limit = 10.;      // curent limit value

// 전류 변수
volatile double g_Ccur;            // Current
volatile double g_Cdes;            // desired Current
volatile double g_Cerr;            // 현재의 Current error
volatile double C_limit = 1.;      // curent limit value

/////////////////////////////////////////////////
/////////////////////////////////////////////////

volatile double g_ADC;
volatile int g_SendFlag = 0;
volatile int g_Direction;

volatile int cur_control = 0;
volatile double g_vel_control;
volatile double g_pos_control;
volatile unsigned char g_TimerCnt;

volatile unsigned char checkSize;
volatile unsigned char g_buf[256], g_BufWriteCnt, g_BufReadCnt;

volatile Packet_t g_PacketBuffer;
volatile unsigned char g_PacketMode;
volatile unsigned char g_ID = 1;

/////////////////////////////////////////////////
//////////      Control Variables      //////////
/////////////////////////////////////////////////
// PID gain Tuning value
volatile double dt = 0.0005;

// 현재 Control Gain //
volatile double Kp_c = 1.327;      // 1.2075
volatile double Ki_c = 2.2117e+03;
volatile double C_err_sum = 0.; // for integral
volatile double C_seturation = 24; // for Anti-Windup

volatile double Kp_v = 1.511;
volatile double Ki_v = 41.159;
volatile double V_err_sum = 0.; // for interal
volatile double V_seturation = 0; // for Anti-Windup

volatile double Kp_p = 4.812;
volatile double Kd_p = 0.127;
/////////////////////////////////////////////////
/////////////////////////////////////////////////


// Motor Speed Control
void SetDutyCW(double v) {
   while(TCNT1  == 0);

   int ocr = v * (200. / 24.) + 200;
   
   if(ocr > OCR_MAX)      ocr = OCR_MAX;
   else if(ocr < OCR_MIN)   ocr = OCR_MIN;
   
   OCR1A = OCR3B = ocr + 8;      //1 H
   OCR1B = OCR3A = ocr - 8;      //1 L
}


void InitLS7366() {
   PORTB = 0x00;
   SPI_MasterSend(SELECT_MDR0 | WR_REG);
   SPI_MasterSend(X4_QUAD | FREE_RUN | DISABLE_INDEX | SYNCHRONOUS_INDEX | FILTER_CDF_1);
   PORTB = 0x01;
   
   PORTB = 0x00;
   SPI_MasterSend(SELECT_MDR1 | WR_REG);
   SPI_MasterSend(FOUR_BYTE_COUNT_MODE | ENABLE_COUNTING);
   PORTB = 0x01;
   
   PORTB = 0x00;
   SPI_MasterSend(SELECT_CNTR | CLR_REG);
   PORTB = 0x01;
}


// Get current
int getADC(char ch) {
   ADMUX = (ADMUX & 0xf0) + ch;
   ADCSRA |= 0x40;
   while(!(ADCSRA & 0x10));
   return ADC;
}



ISR(USART0_RX_vect) {
   g_buf[g_BufWriteCnt++] = UDR0;
}


ISR(TIMER0_OVF_vect){            // Control period: 2ms
   TCNT0 = 256 - 125;
   
   //Read LS7366
   int32_t cnt;
   
   PORTC = 0x01;
   
   g_ADC = getADC(0);
   
   PORTB = 0x00;
   SPI_MasterSend(SELECT_OTR | LOAD_REG);
   PORTB = 0x01;
         
   PORTB = 0x00;
   SPI_MasterSend(SELECT_OTR | RD_REG);
   cnt = SPI_MasterRecv();      cnt = cnt << 8;
   cnt |= SPI_MasterRecv();   cnt = cnt << 8;
   cnt |= SPI_MasterRecv();   cnt = cnt << 8;
   cnt |= SPI_MasterRecv();
   PORTB = 0x01;
   g_Cnt = -cnt;
   
   PORTC = 0x03;
   
   // Read Current Angle [rad]
   // radian to degree (PI*360)
   g_Pcur = (g_Cnt / (4096. * 81.)) * 2 * M_PI;
   
   // 360도 예외처리
   if (g_Pcur > 360.) g_Pcur -= 360.;
   else if (g_Pcur < -360.) g_Pcur += 360.;
   
   if ((g_TimerCnt % 10) == 0) {
      g_Vcur = (g_Pcur - g_Pvcur) / (dt * 10.0);
      g_Pvcur = g_Pcur;
   }
   
   // Read Current
   g_ADC = getADC(0);
   g_Ccur = -( ((g_ADC / 1024. * 5.) - 2.488) * 10.);
   
   //////////////////////////////////////////////////
   //////////          Controller          //////////
   //////////////////////////////////////////////////
   
   // Position Control // PD Controller
   if((g_TimerCnt % 100) == 0){
      g_Perr = g_Pdes - g_Pcur;
      while(g_Perr > 180 * M_PI / 180.){
         g_Perr -= 360 * M_PI / 180.;
      }
      while(g_Perr < -180 * M_PI / 180.){
         g_Perr += 360 * M_PI / 180.;
      }
      g_pos_control = Kp_p*g_Perr + Kd_p*(g_Perr-g_Pperr)/(dt * 100); // PD Controll
      g_Pperr = g_Perr; // 현재 에러값 저장
      
      if(g_pos_control > 642.){
         g_pos_control = 642.;
      }
      else if(g_pos_control < -642.){
         g_pos_control = -642.;
      }
      // User Angluar Velocity Saturation
      if(g_pos_control > V_limit){
         g_pos_control = V_limit;
      }
      else if(g_pos_control < -V_limit){
         g_pos_control = -V_limit;
      }
      g_TimerCnt = 0;

   }
   
   /////////////////////////////////////
   // Velocity Control // PI Controller
   if((g_TimerCnt % 10) == 0){
      g_Vdes = g_pos_control;
      g_Verr = g_Vdes - g_Vcur;
      g_vel_control = g_Verr * Kp_v + V_err_sum * Ki_v * (dt *10 ); // PI Controll
      V_err_sum += g_Verr;
      
      
      // 모터 속도 limit
      if(g_vel_control > 2.08){
         V_err_sum -= (g_vel_control - 2.08) * 1. / Kp_v / 3.;
         g_vel_control = 2.08;
      }
      else if(g_vel_control < -2.08){
         V_err_sum -= (g_vel_control + 2.08) * 1. / Kp_v / 3.;
         g_vel_control = -2.08;
      }
      
      // Saturation을 이용한 오차 누적 방지
      // Anti-Windup
      V_seturation = g_Vdes;
      if(V_err_sum > V_seturation) V_err_sum = V_seturation;
      if(V_err_sum < -V_seturation) V_err_sum = -V_seturation;
      
      // 모터의 최대 Current를 넘지 않게 하기 위함
      if(g_vel_control > C_limit){
         g_vel_control = C_limit;
      }
      else if(g_vel_control < -C_limit){
         g_vel_control = -C_limit;
      }
   }

   ///////////////////////////////////
   // Current Control // PI Controller
   if((g_TimerCnt % 1) == 0){
      g_Cdes = g_vel_control;
      g_Cerr = g_Cdes - g_Ccur;
      cur_control = g_Cerr * Kp_c + C_err_sum * Ki_c * dt; // PI Controll
      
      cur_control += g_Vcur * Ke; // 전향보상
      C_err_sum += g_Cerr;
      
      // Saturation을 이용한 오차 누적 방지
      // Anti-Windup
      if(cur_control > 24){
         C_err_sum -= (g_vel_control - 24.) * 1. / Kp_c / 3.;
         g_vel_control = 24.;
      }
      else if(g_vel_control < -24){
         V_err_sum -= (g_vel_control + 24.) * 1. / Kp_c / 3.;
         g_vel_control = -24.;
      }
   }

   //
   g_TimerCnt++;
   SetDutyCW(cur_control);
   
   /////////////////////////////////////////
   
   g_SendFlag++;
}


int main(void){
   
   Packet_t packet;
   packet.data.header[0] = packet.data.header[1] = packet.data.header[2] = packet.data.header[3] = 0xFE;
   
   InitIO();
   
   //Uart
   InitUart0();
   
   //SPI
   InitSPI();
   
   //Timer
   InitTimer0();
   InitTimer1();
   InitTimer3();


   TCNT1 = TCNT3 = 0;
   SetDutyCW(0.);
   
   //ADC
   InitADC();
   
   //LS7366
   InitLS7366();
   
   //TCNT3 = 65536 - 125;
   TCNT0 = 256 - 125;
   sei();

   unsigned char check = 0;
   
   while (1) {
      PORTA = 0xFF;
      for(;g_BufReadCnt != g_BufWriteCnt; g_BufReadCnt++){
         switch(g_PacketMode){
            case 0:
            if(g_buf[g_BufReadCnt] == 0xFF){
               checkSize++;
               if(checkSize == 4){
                  g_PacketMode =1;
               }
            }
            else{
               checkSize = 0;
            }
            break;
            case 1:
            
            g_PacketBuffer.buffer[checkSize++] = g_buf[g_BufReadCnt];
            
            if(checkSize == 8){
               
               if(g_PacketBuffer.data.id == g_ID){
                  g_PacketMode = 2;
                  
               }
               else{
                  g_PacketMode = 0;
                  checkSize = 0;
               }
            }
            break;
            
            case 2:
            g_PacketBuffer.buffer[checkSize++] = g_buf[g_BufReadCnt];
            check += g_buf[g_BufReadCnt];
            
            if(checkSize == g_PacketBuffer.data.size){
               if(check == g_PacketBuffer.data.check){
                  switch(g_PacketBuffer.data.mode){
                     case 2:
                     g_Pdes =g_PacketBuffer.data.pos /1000.;
                     V_limit=g_PacketBuffer.data.velo /1000.;
                     C_limit=g_PacketBuffer.data.cur /1000.;
                     break;
                  }
               }
               check = 0;
               g_PacketMode = 0;
               checkSize = 0;
               
            }
            else if(checkSize > g_PacketBuffer.data.size || checkSize > sizeof(Packet_t)){
               //TransUart0('f');
               check = 0;
               g_PacketMode =0;
               checkSize = 0;
            }

         }
      }
      if(g_SendFlag > 19){
         g_SendFlag = 0;

         packet.data.id = g_ID;
         packet.data.size = sizeof(Packet_data_t);
         packet.data.mode = 3;
         packet.data.check = 0;
         
         // send data
         packet.data.pos=g_Pcur * 1000;
         packet.data.velo=g_Vcur * 1000;
         packet.data.cur=g_Ccur * 1000;

         for(int i=8; i<sizeof(Packet_t); i++)
         packet.data.check += packet.buffer[i];

         for(int i=0; i<packet.data.size; i++){
            TransUart0(packet.buffer[i]);
         }
      }
   }
   
}
