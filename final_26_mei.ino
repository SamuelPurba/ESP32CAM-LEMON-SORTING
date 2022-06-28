#define JK 9// JERUK KUNING
#define JH 10// JERUK HIJAU

#include <Servo.h>

Servo SJK;// SERVO JERUK KUNING
Servo SJH;// SERVO JERUK KUNING

int kuning = 0;
int hijau = 0;

//[2] Deklarasi
//[2a]Deklarasi library
      #include <LiquidCrystal_I2C.h> //library LCD I2C
    
//[2b]Deklarasi program LCD
      LiquidCrystal_I2C lcd(0x27, 16, 2); //set-up LCD

//[2c]Deklarasi Program Motor Encoder
      #define ENCODEROUTPUT 823 //pulse per rotartion (pulsa yang dihasilkan encoder, dari datasheet)
//[2d]Deklarasi pin yang digunakan
    const int HALLSEN_A = 2; 
      const int HALLSEN_B = 4;// Hall sensor A (Encoder Output A) dihubungkan ke pin 2 (external interrupt)
      const int MOTOR1A = 8;
      const int MOTOR1B = 9;
      const int EN1 = 10;//EN1 dihubungkan ke pin 11 (jika ingin mengatur kecepatan motor)
      
      volatile long encoderValue = 0; //The sample code for driving one way motor encoder

      int interval = 100;
      long previousMillis = 0;
      long currentMillis = 0;
      int rpm = 0;
//[2e]Deklarasi SV, PV, MV
      float SV, PV;
      int MV;
//[2f]Deklarasi perhitungan PID Control
      float Kp,et,et_1,Ti,Ki,Kd,edif,Td,P,I,D;
      float eint, eint_1, eint_update;
//[2g]Deklarasi perhitungan filter 
      float PVf, PVf_1, fc, RC, a, ts;
//[2h]Deklarasi variable time sampling
      unsigned long t;
      double t_1, Ts;
//[2i]Deklarasi untuk plotting
      float interval_elapsed, interval_limit;

String cmd1 = "";

String skuning = " KUNING ";
String shijau = " HIJAU ";

void signalsetup(){
  
  pinMode(JK, INPUT);
  pinMode(JH, INPUT);

  digitalWrite(JK, LOW);
  digitalWrite(JH, LOW);
}

void servosetup(){
  
  SJK.attach(11);
  SJH.attach(12);
}
  
void setup() {
//[3] Setup untuk sistem

      signalsetup();
      servosetup();

      Serial.begin(9600);
      EncoderInit();//memanggil program dari fungsi EncoderInit()
      pinMode(HALLSEN_A, INPUT);
      pinMode(HALLSEN_B, INPUT);
      
//[3b]Set pin yang dipakai
      pinMode(MOTOR1A, OUTPUT);
      pinMode(MOTOR1B, OUTPUT);
      pinMode(EN1, OUTPUT);

//[3c]Set program motor DC
      digitalWrite(MOTOR1A, HIGH);
      digitalWrite(MOTOR1B, LOW);

      encoderValue = 0;
      previousMillis = millis();
      
//[3d]Setup paramter filter yang dipakai
      fc=0.00179; //0.00383;//hasil perhitungan menggunakan filter orde 1//sebelumnya 0.00179
      RC=1/(6.28*fc);
      ts=0.01;//diukur manual terlebih dahulu
      a=RC/ts;
      PVf_1=0;
      
//[3e]Setting untuk mengatur durasi tampilan
      interval_elapsed=0;
      interval_limit=0.1;
      t=0;
//[3f]Set parameter kendali
   Kp=7; //6; (Bagi 11) //66; (Awal)
      Ti=4; //3.883; (Kali 11) //0.353 (Awal)
      Td=0.110; //0.97075; (Kali 11) //0.08825 (Awal)
      
      //untuk menghindari infinity 'NAND', ketike Ti=0;
      if(Ti==0){
         Ki=0;}
           else{
            Ki=Kp/Ti;}
            Kd=Kp*Td;
            eint_1=0;//set awal untuk luasan
}

void logicsignal(){
  
  if(kuning != LOW){
    
      SJK.write(180);
      cmd1 = skuning;
  }else{
      SJK.write(0);
  }
  
  if(hijau != LOW){
    
      SJH.write(180);
      cmd1 = shijau;
  }else{
      SJH.write(0);
  }
}

void scansignal(){
  
  kuning = digitalRead(JK);
  hijau = digitalRead(JH);
}

//[4]
void loop() {

  scansignal();
  logicsignal();

//[4a]Baca SV
      SV = analogRead(A0);
      SV =  map (SV, 0,1023, 0,100);
//[4b]Membaca PV 
      PV=rpm;
      //Program Encoder
      //Update nilai Rpm setiap 1 detik
      currentMillis = millis();
      if(currentMillis - previousMillis > interval){
        previousMillis = currentMillis;
        // Revolutions per minute (RPM) =
        // (total encoder pulsa in 1s/motor encoder output) x 60 s
        rpm = (float)(encoderValue * 600 / ENCODEROUTPUT); 
        // Monitor hanya diupdate ketika ada bacaan (rpm > 0)
        if(rpm < 0) {
          rpm=0;
        }
        else{
          rpm=rpm;
        } 
          encoderValue = 0;     
       }

//[4c]Filter PV
      PVf=(PV+a*PVf_1)/(a+1);
      PVf_1=PVf;//untk perhitungan selanjutnya
//[4d]Hitung error, e=SV-PVf
      et=SV-PVf;
//[4e]Menghitung bagian P
      P=Kp*et;
//[4f]Menghitung bagian I dan D
      eint_update=((et+et_1)*Ts)/2;
      eint=eint_1+eint_update;
      I=Ki*eint;
      edif=(et-et_1)/0.01;
      D = Kd*edif;
//[4g]Menghitung bagian PID  
      MV=P+I+D;     
      //Membatasi keluaran MV agar tdk lebih dari spesifikasi arduino (PWM=0 s.d 255)
      if(MV >255){
        MV=255;}
      else if(MV<0){
        MV=0;}
      else{
        MV=MV;}
      //Menuliskan MV ke pin output, disini menggunakan pin 10
      analogWrite(10, MV);
//[4h]Menghitung Ts
      t_1 = t;
      t = millis();
      Ts = (t - t_1)/1000; //proses perhitungan Time Sampling [Ts] sudah dalam second
//[4i]Menghitung waktu elapsed untuk enentukan kapan nilai di display
      interval_elapsed = interval_elapsed + Ts; //menjumlahkan besarnya Ts secara terus menerus.
//[4j]Cek hasil penjumlahan Ts akan di cek apakah sudah sama/lebih dari batas interval
      if(interval_elapsed >= interval_limit){
//[4k]tampilkan di display plotter dan LCD
      Serial.print(SV);
      Serial.print(" ");
      Serial.print(PVf);
      Serial.print(" ");
      Serial.print(0);
      Serial.print(" ");
      Serial.println(100);
   
       
//[4l]reset nilai IE
      interval_elapsed=0; 
      }//setelah ditampilkan agar perhitungan diulang dari nol lagi

      else {interval_elapsed = interval_elapsed;}

//[4m]Set untuk perhitungan selanjutnya
      et_1=et;
      eint_1=eint;
} 
//[5] Fungsi Encoder Init
      void EncoderInit(){
         attachInterrupt(digitalPinToInterrupt(HALLSEN_A), updateEncoder, RISING);
      }
     //Menambahkan encoderValue dengan 1, setiap encoder mendeteksi sinyal naik(rising)
     //dari hall sensor A
     void updateEncoder(){
        encoderValue++;
     }
