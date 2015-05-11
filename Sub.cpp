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

int main(void){   
  DualShock3 controller;
  MotorSerial ms;

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

  //プルダウン
  pullUpDnControl(In,PUD_DOWN);
  pullUpDnControl(LSIn, PUD_DOWN);

  digitalWrite(MobLED, 1);
	
  if(!controller.connectedCheck()){
    cout << "Error!" << endl;
    return 0;
  }

  //STARTボタンが押されるまでloopする	
	UPDATELOOP(controller, !(controller.button(START) && controller.button(CROSS))){ 	
    
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
  }
  digitalWrite(SMotor1, 0);
  digitalWrite(SMotor2, 0);
  digitalWrite(SMotor3, 0);
  digitalWrite(Out, 0);
  digitalWrite(MobLED, 0);
  
  return 0;
}
