#include <iostream>
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "RasPiDS3/RasPiDS3.hpp"
#include "RasPiMS/RasPiMS.hpp"
#include <cstdio>

using namespace std;
using namespace RPDS3;
using namespace RPMS;

// ロータリーエンコーダ用割り込みピンの設定
  // ローラー上
  const int rot_pinA = 12;
  const int rot_pinB = 25;
  signed long int rot_countA = 0;
  signed long int rot_countB = 0;
  // ローラー下
  const int rot_pinC = 20;
  const int rot_pinD = 18;
  signed long int rot_countC = 0;
  signed long int rot_countD = 0;

// 割り込み前のA相B相それぞれの値を保持
  // ローラー上
  bool oldPinA = false;
  bool oldPinB = false;
  // ローラー下
  bool oldPinC = false;
  bool oldPinD = false;

// 速度を検出するための時間を計測する変数
int time1 = 0;
int time2 = 0;

void rotary_changedPinUP(void);
void rotary_changedPinDOWN(void);

int main(void){   
  DualShock3 controller;
  MotorSerial ms;

  int wiringPiSetup();
  wiringPiSetupSys();

 /*-------GPIOピン割り当て-------*/
  //装填完了用LED
  int LED = 5;
  //近距離用モーター
  int SMotor1 = 17;
  int SMotor2 = 27;
  int SMotor3 = 22;
  //Raspberry Pi相互通信用
  int In = 23;
  int Out = 24;
  //リミットスイッチ
  int LSIn = 19;
  //モバイルバッテリー電源確認用LED
  int MobLED = 13;
  /*-------割り当てここまで-------*/

  double STICK;
	int Rcount = 0;
  int Flag;
  int KAITEN; //double
  int KAITEN_A;
  int KAITEN_O;
  int RollFlag = -1;
  int LEFTFlag;

  try{
		ms.init();
	}
	catch(const char *str){
    return -1;
	}

  pinMode(LED, OUTPUT);
  pinMode(SMotor1, OUTPUT);
  pinMode(SMotor2, OUTPUT);
  pinMode(SMotor3, OUTPUT);
  pinMode(In, INPUT);
  pinMode(Out, OUTPUT);
  pinMode(LSIn, INPUT);
  pinMode(MobLED, OUTPUT);
  pinMode(rot_pinA, INPUT);
  pinMode(rot_pinB, INPUT);
  pinMode(rot_pinC, INPUT);
  pinMode(rot_pinD, INPUT);

  //プルダウン
  pullUpDnControl(In,PUD_DOWN);
  pullUpDnControl(LSIn, PUD_DOWN);
  pullUpDnControl(rot_pinA, PUD_UP);
  pullUpDnControl(rot_pinB, PUD_UP);
  pullUpDnControl(rot_pinC, PUD_UP);
  pullUpDnControl(rot_pinD, PUD_UP);

  wiringPiISR(rot_pinA, INT_EDGE_BOTH, rotary_changedPinUP);
  wiringPiISR(rot_pinB, INT_EDGE_BOTH, rotary_changedPinUP);
  wiringPiISR(rot_pinC, INT_EDGE_BOTH, rotary_changedPinDOWN);
  wiringPiISR(rot_pinD, INT_EDGE_BOTH, rotary_changedPinDOWN);

  digitalWrite(MobLED, 1);
	
  if(!controller.connectedCheck()){
    cout << "Error!" << endl;
    return 0;
  }

  //STARTボタンが押されるまでloopする	
	UPDATELOOP(controller, !(controller.button(START) && controller.button(CROSS))){ 	
    
    int time_diff;
    time2 = micros();

    //可変抵抗の値の処理
    STICK = controller.stick(RIGHT_Y) + 128;
    KAITEN = 0.68 * STICK + 30;

    //ローラー微調整
    KAITEN_A = -(controller.stick(LEFT_Y));
    if(KAITEN_A >= -130 && KAITEN_A < -100)
      KAITEN -= 5;
    if(KAITEN_A >= -100 && KAITEN_A < -75)
      KAITEN -= 4;
    if(KAITEN_A >= -75 && KAITEN_A < -50){
      KAITEN -= 3;
    }if(KAITEN_A >= -50 && KAITEN_A < -25){
      KAITEN -= 2;
    }if(KAITEN_A >= -25 && KAITEN_A < 0){
      KAITEN -= 1;
    }if(KAITEN_A > 0 && KAITEN_A < 25){
      KAITEN += 1;
    }if(KAITEN_A >= 25 && KAITEN_A < 50){
      KAITEN += 2;
    }if(KAITEN_A >= 50 && KAITEN_A < 75){
      KAITEN += 3;
    }if(KAITEN_A >= 75 && KAITEN_A < 100){
      KAITEN += 4;
    }if(KAITEN_A >= 100 && KAITEN_A < 130){
      KAITEN += 5;
    }
    
    if(KAITEN <= 30)
      KAITEN = 30;
    /*if(KAITEN >= 200)
      KAITEN = 200;*/

    //コントローラから非常停止
    if(controller.button(SELECT))
      ms.send(255, 255, 0, true);
    //マシン速度を遅くする
    if(controller.press(RIGHT))
      digitalWrite(Out, 1);
    if(controller.release(RIGHT))
      digitalWrite(Out, 0);
    //近距離
    if(controller.press(SQUARE))
      digitalWrite(SMotor1, 1);
    if(controller.press(TRIANGLE))
      digitalWrite(SMotor2, 1);
    if(controller.press(CIRCLE))
      digitalWrite(SMotor3, 1);
    if(controller.release(SQUARE) || controller.release(TRIANGLE) || controller.release(CIRCLE)){
      digitalWrite(SMotor1, 0);
      digitalWrite(SMotor2, 0);
      digitalWrite(SMotor3, 0);
    }
    //装填(上)
    if(!(digitalRead(LSIn))){
      digitalWrite(LED, 1);
      Flag = false;
    }else{
      digitalWrite(LED, 0);
      Flag = true;
    }
    if(Flag){
      if(controller.press(UP))
        ms.send(8, 2, -200, true);
    }
    if(!Flag){
      if(LEFTFlag != 1)
        ms.send(8, 2, 0, true);
    }
    //装填(下)
    if(controller.press(LEFT)){
      ms.send(8, 2, 200, true);
      LEFTFlag = 1;
    }
    if(controller.release(LEFT)){
      ms.send(8, 2, 0, true);
      LEFTFlag = 0;
    }
    //ローラー
    if(digitalRead(In) == true){
      if(KAITEN != KAITEN_O){
        cout << KAITEN << endl;
        ms.send(8, 3, KAITEN);
        ms.send(13, 3, -KAITEN);
        KAITEN_O = KAITEN;      //値を保存
      }
      RollFlag = -1; 
      Rcount = 0;
    }
    if(digitalRead(In) == false){
      Rcount++;
      if(RollFlag != 1){
        if(Rcount > 3){
          ms.send(8, 5, 0);
          ms.send(13, 5, 0);
          RollFlag = 1;
          KAITEN_O = 0;
        }
      }
    }
    //ロータリーエンコーダ読み取り
    time_diff = time2 - time1;
    if(time_diff > 1000){
      cout << rot_countA << " " << rot_countB << " " << rot_countC << " " << rot_countD << endl;
      rot_countA = 0;
      rot_countB = 0;
      rot_countC = 0;
      rot_countD = 0;
      time1 = micros();
    }
  }
  digitalWrite(SMotor1, 0);
  digitalWrite(SMotor2, 0);
  digitalWrite(SMotor3, 0);
  digitalWrite(Out, 0);
  digitalWrite(MobLED, 0);
  
  return 0;
}

void rotary_changedPinUP(void){

  bool sigPinA = digitalRead(rot_pinA);
  bool sigPinB = digitalRead(rot_pinB);

  if(sigPinA) rot_countA++;
  if(sigPinB) rot_countB++;

  oldPinA = sigPinA;
  oldPinB = sigPinB;
}

void rotary_changedPinDOWN(void){

  bool sigPinC = digitalRead(rot_pinC);
  bool sigPinD = digitalRead(rot_pinD);

  if(sigPinC) rot_countC++;
  if(sigPinD) rot_countD++;

  oldPinC = sigPinC;
  oldPinD = sigPinD;
}
