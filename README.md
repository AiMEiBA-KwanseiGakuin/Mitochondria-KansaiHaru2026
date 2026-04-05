# Mitochondria-KansaiHaru2026
関西春ロボ2026の機体(Mitochondria)のコード整理用リポジトリ  
(このarchiveブランチは大会当日に使用したコードとその仕様をまとめています。)

## 概要
- DualShock4+ESP32でロボットの操縦を行う。  
(保険として,iPhone/Android端末およびPCなどのシリアル通信が可能な機器からの操縦も可能)

## 実行環境
- Arduino IDE ver:2.3.8
- ESP32 ver:4.1.8
- RemoteXY ver:4.1.8
- PS4Controller ver:1.1.0
- PCA9685.h (https://akizukidenshi.com/catalog/g/g110350/)

## 機体
- 機体は4輪オムニホール+ラックアンドピニオンによるやぐら保持機構、サーボモーター4つからなるリング保持アームで構成される。
- DCモーターはツカサモーター+IBT_2、サーボモーターはSG-90とMG996+PCA9685、電源としてDCモーターにはレスコンバッテリー(LeFe12V)、マイコンとサーボモーターは乾電池4本直列の電池ボックス各1個を使用

## 使用したGPIOピン
- 足回り:18,5,2,15,14,12,26,27
- やぐら:25,33
- PCA9685:21,22
- LED:32,0,4

## 操縦(DualShock4)
- 足回りは左ジョイスティックによる8方向への平行移動とR2/L2ボタンによる旋回、Shareボタンによる速度切り替え(チャタリングあり)
- やぐら保持機構は左右ボタンで開閉
- リング保持アームは右ジョイスティックで手首のY･Z軸を移動、〇/✕ボタンで手首関節の角度を調節、R1/L1ボタンでハンド部の開閉
  加えて、△/☐ボタンとその同時押しによるワーク回収位置とワークシュート位置、待機位置への移動
- コントローラー上部のパッドを押し込むことによりコントローラーからの入力を遮断(通信そのものは維持して再度操縦状態への切り替えも可能)(チャタリングあり)
- Share/Optionsボタン同時押しでモーター出力を止め、LEDを点滅(アピール用)

## プログラム解説
バカ丁寧なコメントを書いたので見ればだいたい何やってるかはわかるはず...?
### 設計思想
- どこに何があるかわかるコードを書く！
  →入力と出力をそれぞれファイルに分けて管理
- 土壇場の仕様変更に対応しやすくする
  →操縦方式や操作ロジックはメインに残して変更を容易に
- 機能単位での流用をやりやすくする
  →適度な抽象化を心がける(実際できてるかどうかは納期との兼ね合いもあって怪しい)

### ファイル構成
Final/  
　├─Final.ino  
　 |　　　//メインプログラムのファイル  
　├─SwitchMode.h  
　 |　　　//操縦モード切替用  
　├─Rimocon_RemoteXY.h  
　 |　　　//RemoteXYの設定用  
　├─Rimocon_Serial.h  
　 |　　　//シリアル通信の入力受け取り用  
　├─OmuniLeg.h  
　 |　　　//足回りとやぐら機構制御用  
　└─IKArm.h  
　  　　　//アーム制御用  
(ツリーの書き方の正解わからんすぎる...)

### 定数
```cpp
//--出力用定数--//
constexpr int DC_default_speed = 150; //             櫓用アームの回転速度(足回りは255)[n/255]
constexpr float leg_motor_gains[] = {1, 1, 1, 1}; // 足回りモーターの速度定数[最大のn倍]
constexpr int SERVOMAX = 470, SERVOMIN = 120; //     PCA9685に渡す周波数のレンジ(基本変更しない)
constexpr float arm_speed = 1;//1.25 //              アーム手首位置の移動速度[n*0.6cm(←目安)]
constexpr int wrist_speed = 3; //                    アーム手首角度の回転速度[degree]
constexpr int finger_speed = 6; //                   ハンドの回転速度[degree]
constexpr int led_power = 150; //                    動作チェック用LEDの強さ

constexpr float arm_pos_init[] = { 45.08/4,  80.88/4, 0.00}; // アーム待機位置x,yと手首角度[degree]
constexpr float arm_pos_pick[] = {162.24/4, -85.44/4, 2.72}; // ワーク拾う位置x,yと手首角度[degree]
constexpr float arm_pos_drop[] = {-206.56/4, 126.6/4, 2.54}; // ワークセット位置x,yと手首角度[degree]

//--環境依存定数--//
constexpr int DCpins[]={ // IBT_2用信号線のピン
//  FR      BR      BL      FL    yagura 
   2, 15, 14, 12, 26, 27, 18,  5, 25, 33
};
constexpr int Channels[] = { // PCA9685の使用チャンネル(根本→手先順)
  0, 2, 4, 6
};
constexpr int ArmLength[] = {36, 26, 21}; // アーム関節長さ(3個目は不使用)
constexpr int InitalAngle[] = {0, 0, 90}; // アームのサーボ取付角度(完全折り畳み時が0°)
constexpr int LEDpins[]={32, 4, 0}; //       動作チェック用LEDのピン(接続,アーム閾値,低速走行)
```
### 変数
```cpp
//--入力用変数--//
bool connection_flag; //                      接続確認用
int leg_joystick_x, leg_joystick_y; //        足回り方向指定
int leg_direc; //                             足回り方向指定(ジョイスティック以外のUI用)
bool leg_button_R, leg_button_L; //           旋回方向
bool leg_button_shift; //                     低速走行
bool yagura_R, yagura_L; //                   櫓用アーム開閉
int arm_joystick_x, arm_joystick_y; //        アーム手首位置操作
bool arm_zplus, arm_zminus, arm_yplus, arm_yminus;
  //                                          アーム手首位置操作(ジョイスティック以外のUI用)
bool arm_button_UP, arm_button_DOWN; //       アーム手首角度調節
bool arm_button_init, arm_button_pick, arm_button_drop;
  //                                          アームの指定位置コマンド
bool finger_button_UP, finger_button_DOWN; // ハンドの開閉
```
### 関数,構造体,クラスなど
#### setup関数
```cpp
void setup(){
	// デバッグ兼シミュレータ用シリアル通信 (本番ではコメントアウトOK)
  Serial.begin(115200);

  //--init controller--//
  // 制御方法に応じてコメントアウトを切り替える
  PS4.begin(MAC_PS4CON);
  //RemoteXY_Init();
  //Serial2.begin(115200,SERIAL_8N1, 16, 17);
  
  //--init DCmotors--//
  // DCモーターの初期化
  for(int pin: DCpins){
    ledcAttach(pin,12800,8);
    ledcWrite(pin,0);
  }

  //--init Arm--//
  // アーム制御structの初期化関数
  sakuarm.begin();//true); // 引数にtrueを渡すとPCA9685なしで動作確認可能

  //--init LED--//
	// 動作チェック用LEDの初期化(ちょっと光る)
  for(int pin: LEDpins){
    ledcAttach(pin,12800,8);
    ledcWrite(pin,50);
  }
}
```
#### loop関数
```cpp
void loop(){
  //--get inputs--//
  // 制御方法に応じてコメントアウトを切り替える
  PS4Input();
  //RemoteXYEngine.handler(); RemoteXYInput();
  //connection_flag = SerialInput();

  //--process logic--//
  if(connection_flag){
  //-manage omuni-//
  // 足回り制御(_direcXと_direcYで進行方向を指定)
    if(sq(leg_joystick_x) + sq(leg_joystick_y) > sq(range_ignoreLstick)){
      setdirection(atan2(leg_joystick_y, leg_joystick_x), _direcX, _direcY);
    }
    //if(leg_direc){ // ジョイスティック以外のUI対応版は制作中..→コメントアウトで切り替えを想定
    //}
    else{
      _direcX = 0; _direcY = 0;
    } // →set direcX/Y
    if(leg_button_shift){ // 低速モード切替
      leg_slow = !leg_slow;
      ledcWrite(LEDpins[2], leg_slow *led_power);
    }
    driveomuni(_direcX, _direcY, leg_button_R-leg_button_L, 255 *(leg_slow+0.5)); // モーターの駆動関数

  //-manage yagura-//
    drivemotor(YAGURAARM, DC_default_speed *(yagura_L - yagura_R));

  //-manage ikarm-//
    if(arm_button_init){ // 待機位置ボタンの検知
      setWrist(sakuarm.Ik, arm_pos_init[0], arm_pos_init[1]);
      sakuarm.servoAngle[2] = degrees(arm_pos_init[2]);
    }
    if(arm_button_pick){ // 拾い位置ボタンの検知
      setWrist(sakuarm.Ik, arm_pos_pick[0], arm_pos_pick[1]);
      sakuarm.servoAngle[2] = degrees(arm_pos_pick[2]);
    }
    if(arm_button_drop){ // セット位置ボタンの検知
      setWrist(sakuarm.Ik, arm_pos_drop[0], arm_pos_drop[1]);
      sakuarm.servoAngle[2] = degrees(arm_pos_drop[2]);
    }
    if(sq(arm_joystick_x) + sq(arm_joystick_y) > sq(range_ignoreRstick+10)){
     // マニュアル操作
      if(sakuarm.moveWrist(arm_speed *0.01*(arm_joystick_x), // should be changed about  arm_speed*0.01*..
                        arm_speed *0.01*(arm_joystick_y))){// ↖実はジョイスティックの位置で移動速度を変えれる
        ledcWrite(LEDpins[1],led_power); // 可動域以上に動かそうとした場合はLED点灯で警告
      }else{
        ledcWrite(LEDpins[1],0);
      }
    }else{
      sakuarm.moveWrist(0,0); // これは消すな(ボタンによる位置指定の後の角度計算に必要)
    }
    sakuarm.rotateHand(wrist_speed*(arm_button_UP - arm_button_DOWN)); // 手首角度調整
    sakuarm.moveFinger(finger_speed*(finger_button_UP - finger_button_DOWN)); // ハンド開閉調整
    sakuarm.updateServos(); // 全サーボを指定した角度に動かす(PCA9685未接続時はコメントアウトすること)

  }else{
    for(int pin: DCpins){ // 未接続時の保険としてモーターはOFFする
      ledcWrite(pin,0);
    }
  }
  /*------------------Pythonアームシミュレータとの通信用
  Serial.print(sakuarm.servoAngle[0]); Serial.print(",");
  Serial.print(sakuarm.servoAngle[1]); Serial.print(",");
  Serial.print(sakuarm.servoAngle[2]); Serial.print(",");
  Serial.print(sakuarm.Ik.wrist[0]); Serial.print(",");
  Serial.println(sakuarm.Ik.wrist[1]);
  //*///-----------------
  ledcWrite(LEDpins[0],connection_flag*led_power); // コントローラー接続時のみLED点灯

  delay(10);
  //RemoteXYEngine.delay(10); // RemoteXYを使うときはこっち
}

```
#### Rimocon_Serial.h
RemoteXY関連の設定部分を隔離したもの。  
BLE(Bluetooth Low Energy)とBluetooth classic (Android版のみ)を切り替える改造をしてある。  
2個目のstructに入力変数の名前と説明が書いてあるので、変更時はここを参照してSwitchMode.hの関数内を書き換える。  
**UI変更時はcopy configulationから#pragma pack(push, 1)と#pragma pack(pop)の間だけを置き換えること！**  
#### Rimocon_RemoteXY.h
シリアル通信への対応用。  
各入力をコンマ区切りで受け取って入力用変数に格納する。実は動かしたことがないのでどんな不具合が潜んでるかわからん。
#### OmuniLeg.h
```cpp
inline void setdirection(float inputAngle, int& direcX, int& direcY);
// else if でジョイスティックの入力角度からXY軸の移動方向(+1/0/-1)を指定するだけの脳筋コード
// 理想を言えばsinとcosでswitchした方が5倍はきれいに書けるが、元のGeminiコード(カス)をリファクタする時にそんな余裕はなかった

inline void drivemotor(int index, int speed = DC_default_speed);
// DCモーター全般の駆動用。indexにファイル上部で指定した名前を入れると読みやすくてよい。
// 櫓用アームはこれを直接呼んで制御してる。

inline void driveomuni(int direcX, int direcY, int turn, int speedVal = DC_default_speed);
// setdirectionで指定したXY軸の移動方向から各モーター分のdrivemotorする関数。
// 一度signを挟むことでどの向きでも同じ速度でモーターが回転する。(合力としての機体の移動速度は知らん←オムニの摩擦による抵抗が絡むため)
```
#### IKArm.h
```cpp
inline float clip2pi(float ang);
 // 便利関数その2　計算後の角度をそのまま扱うと負の値や2π以上になったりして始末が悪いので0~2πに収める
 
struct IK2{
  const int* jointLength; // 関節間距離配列
  const int* initAngle; //   各サーボの取り付け時の角度
  int jointLength_sq[2]; //  計算用にあらかじめ関節間距離の2乗値をもっておく配列
  float wrist[2]; //         手首位置
  float elbow[2]; //         肘位置
  float jointAngle[2]; //    (計算用)関節角度配列
  float distance_sq; //      手首-根本の距離の2乗
  float distance; //         手首-根本の距離
};
inline void setWrist(IK2& ik, float x = Pos_Init[0], float y = Pos_Init[1]);
 // 読んで字の如く手首位置を更新する関数
inline void calcAngle(IK2& ik);
 // 逆運動学による肘・肩関節の角度計算
inline void InitIK(IK2& ik);
 // 初期化関数
inline bool isWristOK(IK2& ik);
 // 指定した手首位置が可動域の外にあるかどうかを判定→LEDで警告
 // (サーボの取り付けミスなどで挙動がおかしい時は可動域外をターゲットした状態での動作を観察すると-
 //-不具合の原因がわかりやすいため、あえて可動域外をターゲットすることも可能にしている)

class Arm{ // 肘と肩以外の関節もまとめて制御した方が良いに決まっているので敢えて2重構造にした。
 public:
  const int* initAngle; // structに渡すだけ
  int servoAngle[4]; //    サーボへの出力角度(肘と肩はIkの値から変換)
  IK2 Ik; //               structのインスタンスをメンバとして持つ
  PCA9685 pwm = PCA9685(0x40); // サーボモータードライバーの操作用
  Arm(const int Lengths[], const int Angles_Init[]); // コンストラクタ(pythonの__init__と同じ)
  void begin(bool isTest = false);//    開始用関数 引数にtrueを渡すとPCA9685なしで動かせる
  bool moveWrist(float dx, float dy);// 手首位置の指定用関数(loopから呼ばれてるのはこっち)
  void rotateHand(int da);//            読んで字の如く手首の回転
  void updateServos();//                全サーボモーターの角度をPCA9685に渡して動かしてもらう(PCA9685未接続時はloopでの呼び出し部分をコメントアウトすること)  
};
```
---
作成者:金栄智治(https://github.com/Tomoooji)
