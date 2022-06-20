#include<TimerOne.h>
#include<TimerThree.h>
#include<TimerFive.h>
#define STEP_THE1 22
#define DIR_THE1 20
#define E_STEP1 18
#define MAX_THE1 135
#define MIN_THE1 -135
#define STEP_THE2 28
#define DIR_THE2 26
#define Enable_step2 24
#define maxthe2 70
#define minthe2 -180

volatile long  xung_phat_the2, xung_dat_the2;
volatile long  xung_phat_the1, xung_dat_the1;
volatile double  vi_tri_thuc_the2,vi_tri_dat_the2,vi_tri_ke_tiep2,goc_dat_the2,time_the2;
volatile double  vi_tri_thuc_the1,vi_tri_dat_the1,vi_tri_ke_tiep1,goc_dat_the1,time_the1;
volatile double time_dat, time_dem;
volatile double a1,a2,a3,b1,b2,b3,c1,c2,c3,d1,d2,d3;
volatile bool enable, bu2, bu3;
volatile int sai_so_the1, sai_so_the2, sai_so_the3;
const double gpx_the2 = 1.8/16/(57/11)/(80/30); 
const long PPR_the2 = 200*16*(57/11)*(80/30);
const double gpx_the1 = 1.8/16/(57/11);
const long PPR_the1 = 200*16*(57/11); 
bool pulse2=0;
bool pulse1 = 0;
void setup()
{
  pinMode(DIR_THE2, OUTPUT);
  pinMode(STEP_THE2, OUTPUT);
  digitalWrite(DIR_THE2, LOW);
  digitalWrite(STEP_THE2, LOW);
  digitalWrite(Enable_step2,LOW);
  pinMode(DIR_THE1, OUTPUT);
  pinMode(STEP_THE1, OUTPUT);
  digitalWrite(DIR_THE1, LOW);
  digitalWrite(STEP_THE1, LOW);
  digitalWrite(E_STEP1,LOW);
  vi_tri_thuc_the2 = 0;
  vi_tri_dat_the2 = vi_tri_thuc_the2;
  
  vi_tri_thuc_the1 = 0;
  vi_tri_dat_the1 = vi_tri_thuc_the1;
  
  time_dem = 0;
  time_dat = 0;
  Serial.begin(9600);
  Timer5.attachInterrupt(phat_xung_the2);
  Timer5.stop();
  Timer3.attachInterrupt(phat_xung_the1);
  Timer3.stop();
  Timer1.attachInterrupt(quy_hoach);
  Timer1.stop();
  
    // put your main code here, to run repeatedly:

}
void serialEvent()
{
  if(Serial.available()>2)
  {
    //kitu = Serial.read();
    //if(kitu == 'd')
      // Nhận giá trị từ Serial
      Serial.print("nhap gia tri goc the1: ");
      vi_tri_dat_the1 = Serial.parseFloat();
      Serial.println(vi_tri_dat_the1);
      Serial.print("nhap gia tri goc the2: ");
      vi_tri_dat_the2 = Serial.parseFloat();
      Serial.println(vi_tri_dat_the2);
      Serial.print("nhap tgian:");
      time_dat = Serial.parseFloat();
      enable = true;
      if((vi_tri_dat_the1 < MIN_THE1) || (vi_tri_dat_the2 > MAX_THE1))
      {
        vi_tri_dat_the2 = vi_tri_thuc_the2;
      }
      if((vi_tri_dat_the2 < minthe2) || (vi_tri_dat_the2 > maxthe2))
      {
        vi_tri_dat_the2 = vi_tri_thuc_the2;   
      }
      if((vi_tri_dat_the1 != 0) && (vi_tri_dat_the2 != 0)) enable = false;
      if(time_dat <=0) enable = false;
      if(enable == true)
      {
        time_dem = 0;
        // Tính các hệ số quy hoạch quỹ đạo
        // the1
        a1 = vi_tri_thuc_the1;
        b1 = 0;
        c1 = 3*(vi_tri_dat_the1 - vi_tri_thuc_the1)/(time_dat*time_dat);
        d1 = -2*(vi_tri_dat_the1 - vi_tri_thuc_the1)/(time_dat*time_dat*time_dat);
        // The2
        a2 = vi_tri_thuc_the2;
        b2 = 0;
        c2 = 3*(vi_tri_dat_the2 - vi_tri_thuc_the2)/(time_dat*time_dat);
        d2 = -2*(vi_tri_dat_the2 - vi_tri_thuc_the2)/(time_dat*time_dat*time_dat);
        // The3
        // Chạy quy hoạch
        Timer1.initialize(10000);
        Timer3.initialize(10000);
        bu2 = true;
      }
  }
}
void xuat_chieu_quay(volatile float goc, volatile int chan_chieu)
{
  if(goc >= 0)
  {
    digitalWrite(chan_chieu, HIGH);
    }
  else
  {
    digitalWrite(chan_chieu, LOW);
    } 
  }
//=================================================================
void bu_the2()
{
  sai_so_the2 = int((vi_tri_dat_the2 - vi_tri_thuc_the2)*320/9);
  xuat_chieu_quay(sai_so_the2, DIR_THE2);
  if(abs(sai_so_the2)>0)
  {
    digitalWrite(STEP_THE2,HIGH);
    if(sai_so_the2 > 0)
      vi_tri_thuc_the2 += gpx_the2;
    else
      vi_tri_thuc_the2 -= gpx_the2;
    digitalWrite(STEP_THE2, LOW);
    }
  else
    bu2 = false; 
  }
void quy_hoach()
{
  if(time_dem >= time_dat)
  {
    if(bu2 == false)
    {
      Timer1.stop();
     }
    else 
    {
      bu_the2();
     }   
    }
  else if (time_dem < time_dat)
   {
    time_dem += 0.01;

    vi_tri_ke_tiep1 = a1 + b1*time_dem + c1*(time_dem*time_dem) + d1*(time_dem*time_dem*time_dem);
    goc_dat_the1 = vi_tri_ke_tiep1 - vi_tri_thuc_the1;
    
    vi_tri_ke_tiep2 = a2 + b2*time_dem + c2*(time_dem*time_dem) + d2*(time_dem*time_dem*time_dem);
    goc_dat_the2 = vi_tri_ke_tiep2 - vi_tri_thuc_the2;
    if((goc_dat_the2 != 0) && (goc_dat_the1 != 0 ))
    {
      xuat_chieu_quay(goc_dat_the1, DIR_THE1);
      xung_dat_the1 = round(abs(goc_dat_the1)*PPR_the1/360);
      time_the1 = 10000/xung_dat_the1/2;
      xung_phat_the1 = 0;
      
      xuat_chieu_quay(goc_dat_the2, DIR_THE2);
      xung_dat_the2 = round(abs(goc_dat_the2)*PPR_the2/360);
      time_the2 = 10000/xung_dat_the2/2;
      xung_phat_the2 = 0;
      Timer3.initialize(time_the1);
      Timer5.initialize(time_the2);
      
      }
    }
  }
void phat_xung_the1()
{
  if(xung_phat_the1 >= xung_dat_the1)
  {
    goc_dat_the1 = 0;
    Timer3.stop();
   }
}
 void phat_xung_the2()
{
  if(xung_phat_the2 >= xung_dat_the2)
  {
    goc_dat_the2 = 0;
    Timer5.stop();
    }
  else 
  {
    digitalWrite(STEP_THE2,HIGH);
    xung_phat_the2++;
    if(goc_dat_the2 > 0)
      vi_tri_thuc_the2 += gpx_the2;
    else 
      vi_tri_thuc_the2 -= gpx_the2;
    digitalWrite(STEP_THE2, LOW);
    }
  }
  void loop() {
    Serial.println(vi_tri_thuc_the1);
    Serial.print(vi_tri_thuc_the2);
  
  }
