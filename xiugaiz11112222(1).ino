#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <Otto9.h> //-- Otto Library version 9
#include <Servo.h>
#include <Oscillator.h>

#include <LedControl.h>  // 引入MAX7219驱动库

// 原有舵机控制变量
Otto9 Otto;
Servo leftArm;   // 左手舵机 - 引脚7
Servo rightArm;  // 右手舵机 - 引脚6
const int CMD_EXIT = -1;
const int CMD_HIGH_PRIORITY = 1; // 新增
// 全局变量
int distance;
bool obstacleDetected = false;
int positions[] = {90, 90, 90, 90};
uint8_t servo = 0;
int8_t trims[4] = {0,0,0,0};
bool exitRequested = false; // 退出标志
int currentMode = -1; // 当前运行模式
// MAX7219点阵模块引脚（选择不冲突的空闲引脚）
#define MAX7219_CLK 11   // 时钟引脚（示例：D10）
#define MAX7219_DIN 13   // 数据引脚（示例：D11）
#define MAX7219_CS  12   // 片选引脚（示例：A0，避免与现有引脚冲突）
// 初始化MAX7219控制对象（1个模块）
LedControl lc = LedControl(MAX7219_DIN, MAX7219_CLK, MAX7219_CS, 1);
// 笑脸表情的8x8点阵数据（每行8个像素）
byte smileyFace[8] = {0x0C, 0x12, 0x22, 0x44, 0x44, 0x22, 0x12, 0x0C};
#define N_SERVOS 6 // 共6个舵机

// Servo Pins for Legs and Arms
#define PIN_YL 3  // Left leg
#define PIN_YR 2  // Right leg
#define PIN_RL 5  // Left foot
#define PIN_RR 4  // Right foot
#define PIN_AL 7  // Left arm
#define PIN_AR 6  // Right arm

Oscillator oscillators[N_SERVOS];  // 更改名字，避免与uint8_t冲突
// ULTRASONIC PINs 
#define PIN_Trigger  8  //TRIGGER pin (8)
#define PIN_Echo     9  //ECHO pin (9)
// BUZZER PIN 
#define PIN_Buzzer  13 //BUZZER pin (13)
// SERVO ASSEMBLY PIN  
#define PIN_ASSEMBLY    12   //ASSEMBLY pin (7) LOW = assembly    HIGH  = normal operation


// 命令定义
#define CMD_CALIBRATION 1
#define CMD_MOVE_FORWARD 2      // 指令2改为前进
#define CMD_AVOID_OBSTACLE 3
#define CMD_MOVE_BACKWARD 4     // 指令4改为后退
#define CMD_HAPPY_DANCE 5
#define CMD_ALL_MOVES 6
#define CMD_SING_MARIO 7
#define CMD_JINGLE_BELLS 8
#define CMD_HAPPY_BIRTHDAY 9
#define CMD_FOLLOW 10
#define CMD_RUN 11
#define CMD_ALARM 12
#define CMD_RETRIEVE 13
#define CMD_MEASURE_DISTANCE 14
#define CMD_PLAY_SW 15
#define CMD_EXIT -1
#define CMD_HELP 0
// 显示笑脸表情
void showSmiley() {
  // 循环设置8x8点阵的每个像素
  for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 8; col++) {
      // 从点阵数据中提取对应像素的亮/灭状态
      boolean state = (smileyFace[row] >> (7 - col)) & 1;
      lc.setLed(0, row, col, state);  // 第1个模块（0），行row，列col，状态state
    }
  }
}

// 状态定义
enum ModeState {
  MODE_INIT,
  MODE_RUNNING,
  MODE_COMPLETED,
  MODE_ERROR
};

// 当前模式状态
ModeState currentState = MODE_INIT;

// 设置舵机引脚
void setupServos() {
    oscillators[0].attach(PIN_RR);
    oscillators[1].attach(PIN_RL);
    oscillators[2].attach(PIN_YR);
    oscillators[3].attach(PIN_YL);
    oscillators[4].attach(PIN_AR); // 右臂 90° 平直
    oscillators[5].attach(PIN_AL); // 左臂 90° 平直
}

void homeServos() {
    for (int i = 0; i < N_SERVOS; i++) {
        oscillators[i].SetPosition(90);  // 将舵机归位
        delay(20); // 延迟确保舵机有时间完成归位
    }
    leftArm.write(90);  // 初始状态
    rightArm.write(90); // 初始状态
    delay(500);  // 增加一个延迟，确保所有舵机的动作完成
}

void walkTest(int steps, int T) {
    int A[6]= {15, 15, 30, 30, 30, 30}; // 移动幅度 (右脚 左脚 右腿 左腿 右臂 左臂)
    int O[6] = {0, 0, 0, 0, 0, 0}; // 偏移  (+偏右 -偏左)
    double phase_diff[6] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(90), DEG2RAD(90), DEG2RAD(0), DEG2RAD(0)}; 

    for (int i = 0; i < steps; i++) {
        for (int i = 0; i < N_SERVOS; i++) {
            oscillators[i].SetO(O[i]);
            oscillators[i].SetA(A[i]);
            oscillators[i].SetT(T);
            oscillators[i].SetPh(phase_diff[i]);
        }

        double ref = millis();
        for (double x = ref; x < T + ref; x = millis()) {
            for (int i = 0; i < N_SERVOS; i++) {
                oscillators[i].refresh();
            }
        }
    }
}

// 设置所有4个偏移值
void setTrims(void)
{
  Otto.setTrims(trims[0], trims[1], trims[2], trims[3]);
  Serial.print("Setting trim [");
  for( int x = 0; x < 4 ; x++)
  {
    Serial.print(trims[x]);
    Serial.print(", ");
  }
  Serial.println("]");
}

// 打印帮助信息
void printHelp(void)
{
  Serial.println("\n=== Otto Robot Command Help ===");
  Serial.println("Select a function by entering its number:");
  Serial.println("1. Calibration");
  Serial.println("2. Move Forward");       // 指令2改为前进
  Serial.println("3. Avoid Obstacle");
  Serial.println("4. Move Backward");      // 指令4改为后退
  Serial.println("5. Happy Dance");
  Serial.println("6. All Moves");
  Serial.println("7. Sing Mario Bros");
  Serial.println("8. Jingle Bells");
  Serial.println("9. Happy Birthday");
  Serial.println("10. Follow");
  Serial.println("11. Run");
  Serial.println("12. Alarm");
  Serial.println("13. Retrieve");
  Serial.println("14. Measure Distance");
  Serial.println("15. Play SW");
  Serial.println("\nOther commands:");
  Serial.println("0. Help - Show this menu");
  Serial.println("-1. Exit - Exit current mode");
  Serial.println("");
}



// 命令解析状态
enum ParseState {
  WAITING,      // 等待输入
  RECEIVING_1,  // 已接收第一个数字
  TIMEOUT       // 超时状态
};

// 处理用户输入字符
void processChar(char c)
{
  static String commandBuffer = ""; // 命令缓冲区
  static ParseState parseState = WAITING; // 当前解析状态
  static unsigned long lastInputTime = 0; // 上次输入时间
  static const unsigned long INPUT_TIMEOUT = 500; // 输入超时时间（毫秒）
  
  // 忽略空字符
  if (c == '\n' || c == '\r') return;
  
  // 处理超时状态
  if (parseState != WAITING && millis() - lastInputTime > INPUT_TIMEOUT) {
    // 根据当前状态处理超时
    if (parseState == RECEIVING_1 && commandBuffer.length() == 1) {
      // 一位数命令
      int command = commandBuffer.toInt();
      if (command >= 0 && command <= 9) {
        currentMode = command;
        currentState = MODE_INIT;
        Serial.print("Executing command: ");
        Serial.println(currentMode);
      }
    }
    
    // 重置状态
    commandBuffer = "";
    parseState = WAITING;
    return;
  }
  
  // 状态机处理
  switch(parseState) {
    case WAITING:
      // 开始接收新命令
      commandBuffer = String(c);
      lastInputTime = millis();
      
      // 处理特殊命令
      if (c == 'q' || c == 'Q') {
        currentMode = CMD_EXIT;
        currentState = MODE_INIT;
        Serial.println("Exit command received");
        parseState = WAITING;
        commandBuffer = "";
        return;
      }
      
      if (c == 'h' || c == 'H') {
        printHelp();
        currentMode = -1;
        currentState = MODE_INIT;
        parseState = WAITING;
        commandBuffer = "";
        return;
      }
      
      // 处理负号（可能是-1）
      if (c == '-') {
        parseState = RECEIVING_1;
        return;
      }
      
      // 处理数字输入
      if (isdigit(c)) {
        int command = commandBuffer.toInt();
        
        // 一位数命令立即执行
        if (command >= 0 && command <= 9) {
          // 检查是否可能是两位数命令的第一个数字
          if (command == 1) { // 只有1开头的数字可能组成两位数命令
            parseState = RECEIVING_1;
          } else {
            // 其他数字直接作为一位数命令执行
            currentMode = command;
            currentState = MODE_INIT;
            Serial.print("Executing command: ");
            Serial.println(currentMode);
            commandBuffer = "";
            parseState = WAITING;
          }
        } 
        else {
          // 无效输入，重置
          commandBuffer = "";
          parseState = WAITING;
        }
      } else {
        // 无效输入，重置
        commandBuffer = "";
        parseState = WAITING;
      }
      break;
      
    case RECEIVING_1:
      // 已接收第一个数字（1），等待第二个数字
      commandBuffer += c;
      lastInputTime = millis();
      
      // 处理两位数命令
      if (commandBuffer.length() == 2 && isdigit(commandBuffer.charAt(0)) && isdigit(commandBuffer.charAt(1))) {
        int command = commandBuffer.toInt();
        
        // 检查是否是有效的两位数命令（10-15）
        if (command >= 10 && command <= 15) {
          currentMode = command;
          currentState = MODE_INIT;
          Serial.print("Executing command: ");
          Serial.println(currentMode);
          commandBuffer = "";
          parseState = WAITING;
        } 
        // 无效的两位数命令
        else {
          // 执行第一个数字（1）
          currentMode = 1;
          currentState = MODE_INIT;
          Serial.print("Executing command: ");
          Serial.println(currentMode);
          
          // 继续处理第二个数字（如果是有效的数字）
          if (isdigit(c)) {
            processChar(c); // 递归处理第二个数字
          }
          
          commandBuffer = "";
          parseState = WAITING;
        }
      }
      // 处理-1命令
      else if (commandBuffer == "-1") {
        currentMode = CMD_EXIT;
        currentState = MODE_INIT;
        Serial.println("Exit command received");
        commandBuffer = "";
        parseState = WAITING;
      }
      // 处理无效输入
      else {
        // 执行第一个数字（1）
        currentMode = 1;
        currentState = MODE_INIT;
        Serial.print("Executing command: ");
        Serial.println(currentMode);
        
        commandBuffer = "";
        parseState = WAITING;
      }
      break;
      
    case TIMEOUT:
      // 超时状态，忽略输入
      commandBuffer = "";
      parseState = WAITING;
      break;
  }
}

void runCalibration() {
    static bool initialized = false;
  
    switch(currentState) {
        case MODE_INIT:
            Serial.begin(57600);
            Otto.init(PIN_YL, PIN_YR, PIN_RL, PIN_RR, true, A6, PIN_Buzzer, PIN_Trigger, PIN_Echo);
            printHelp();
            for (int i = 0; i < 4; i++) {
                int servo_trim = EEPROM.read(i);
                if (servo_trim > 128) 
                    servo_trim -= 256;
                trims[i] = servo_trim;
            }
            Serial.println("Getting EEPROM Trim values");
            setTrims();
            Otto._moveServos(10, positions);
            homeServos();  // 校准时，确保舵机归位
            initialized = true;
            currentState = MODE_RUNNING;
            break;
          
        case MODE_RUNNING:
            Otto.home();  // 确保所有舵机都回到初始位置
            while(Serial.available() > 0) {
                processChar(Serial.read());
                if (currentMode == CMD_EXIT) {
                    initialized = false;
                    currentMode = -1;
                    currentState = MODE_INIT;
                    return;
                }
            }
            break;
          
        default:
            initialized = false;
            currentMode = -1;
            currentState = MODE_INIT;
            break;
    }
}
void checkServoPositions() {
    for (int i = 0; i < N_SERVOS; i++) {
        Serial.print("Servo ");
        Serial.print(i);
        Serial.print(" position: ");
        Serial.println(oscillators[i].getPosition());  // 输出每个舵机的实际位置
    }
}






void runMoveForward() {
    static bool initialized = false;
  
    switch (currentState) {
        case MODE_INIT:
            Serial.begin(57600);
            setupServos();
            homeServos();  // 确保舵机从初始位置开始
            Otto.sing(1);  // 播放连接音效
            delay(500);  // 等待舵机归位完成
            initialized = true;
            currentState = MODE_RUNNING;
            break;
          
        case MODE_RUNNING:
            walkTest(4, 800);  // 4步，1000ms每步
            while (Serial.available() > 0) {
                processChar(Serial.read());
                if (currentMode == CMD_EXIT) {
                    initialized = false;
                    currentMode = -1;
                    currentState = MODE_INIT;
                    return;
                }
            }
            break;
      
        default:
            initialized = false;
            currentMode = -1;
            currentState = MODE_INIT;
            break;
    }
}


// 含手臂协调的后退步态
void walkTestBackwardWithArms(int steps, int T) {
    // 幅度配置（索引0-3对应腿部，4-5对应手臂）
    int A[6] = { 
        15, 15,    // 左脚(索引0)，右脚(索引1)
        30, 30,    // 右腿(索引2)，左腿(索引3)
        30, 30     // 右臂(索引4)，左臂(索引5) [新增]
    };
    
    // 相位差配置（关键修改点）
    double phase_diff[6] = {
        DEG2RAD(0),     // 左脚
        DEG2RAD(0),     // 右脚
        DEG2RAD(90),    // 右腿（后退核心）
        DEG2RAD(90),    // 左腿（后退核心）
        DEG2RAD(-90),   // 右臂：与腿部反相摆动[3](@ref)
        DEG2RAD(-90)    // 左臂：与腿部反相摆动[3](@ref)
    };

    // 方向补偿（保持之前的修复）
    const int directionCompensation = -1;

    for(int stepCount = 0; stepCount < steps; stepCount++) {
        // 配置所有6个关节（4腿+2臂）
        for(int servoIdx = 0; servoIdx < 6; servoIdx++) {
            oscillators[servoIdx].SetO(0);
            oscillators[servoIdx].SetA(A[servoIdx]);
            oscillators[servoIdx].SetT(T);
            // 应用方向补偿和相位差
            oscillators[servoIdx].SetPh(phase_diff[servoIdx] * directionCompensation);
        }

        // 非阻塞执行（保持实时响应）
        double startTime = millis();
        while(millis() - startTime < T) {
            // 刷新所有6个舵机
            for(int servoIdx = 0; servoIdx < 6; servoIdx++) {
                oscillators[servoIdx].refresh();
            }
            // 保留串口中断响应
            if(Serial.available()) return;
        }
    }
}


// 后退功能（含手臂协调动作）
void runMoveBackward() {
    static bool initialized = false;
    
    switch(currentState) {
        case MODE_INIT:
            Serial.begin(57600);
            setupServos();
            homeServos();
            initialized = true;
            currentState = MODE_RUNNING;
            Serial.println("BACKWARD MODE WITH ARM MOTION");
            break;
            
        case MODE_RUNNING:
            walkTestBackwardWithArms(10, 800);  // 后退步态+手臂协调
            while(Serial.available() > 0) {
                processChar(Serial.read());
                if(currentMode == CMD_EXIT) {
                    initialized = false;
                    currentMode = -1;
                    currentState = MODE_INIT;
                    Otto.home();
                    return;
                }
            }
            break;
    }
}
// 避障功能
void runAvoidObstacle()
{
  static bool initialized = false;
  
  switch(currentState) {
    case MODE_INIT:
      Otto.init(PIN_YL, PIN_YR, PIN_RL, PIN_RR, true, A6, PIN_Buzzer, PIN_Trigger, PIN_Echo);
      pinMode(PIN_ASSEMBLY, INPUT_PULLUP);
      Otto.sing(S_connection);
      Otto.home();
      delay(500);
      initialized = true;
      currentState = MODE_RUNNING;
      break;
      
    case MODE_RUNNING:
      if (digitalRead(PIN_ASSEMBLY) == LOW)
      {
        Otto.home();
        Otto.sing(S_happy_short);
        delay(500);
      }
      
      if (obstacleDetected)
      {
        Otto.sing(S_surprise);
        Otto.jump(5, 500);
        Otto.sing(S_cuddly);
        for (int i = 0; i < 3; i++)
          Otto.walk(1, 1300, -1);
        delay(500);
        for (int i = 0; i < 3; i++)
        {
          Otto.turn(109, 500, 1);
        
          delay(500);
        }
        obstacleDetected = false; // 重置障碍检测状态
      }
      else
      {
        Otto.walk(1, 600, 1);
        distance = Otto.getDistance();
        if (distance < 15)
          obstacleDetected = true;
        else
          obstacleDetected = false;
      }
      
      while(Serial.available() > 0)
      {
        processChar(Serial.read());
        if (currentMode == CMD_EXIT) {
          initialized = false;
          currentMode = -1;
          currentState = MODE_INIT;
          return;
        }
      }
      break;
      
    default:
      initialized = false;
      currentMode = -1;
      currentState = MODE_INIT;
      break;
  }
}


// 执行所有动作（修改为非阻塞模式，使用状态机）
void runAllMoves()
{
  static int moveStep = 0;
  static unsigned long stepStartTime = 0;
  static const unsigned long STEP_DURATION = 5000; // 每个动作步骤持续时间
  
  switch(currentState) {
    case MODE_INIT:
      Otto.init(PIN_YL, PIN_YR, PIN_RL, PIN_RR, true, A6, PIN_Buzzer, PIN_Trigger, PIN_Echo);
      Otto.sing(S_connection);
      Otto.home();
      delay(50);
      moveStep = 0;
      stepStartTime = millis();
      currentState = MODE_RUNNING;
      break;
      
    case MODE_RUNNING:
      unsigned long currentTime = millis();
      
      // 检查是否需要执行下一步
      if (currentTime - stepStartTime >= STEP_DURATION) {
        moveStep++;
        stepStartTime = currentTime;
      }
      
      // 执行当前动作步骤（按照原allMoves()的顺序）
      switch(moveStep) {
        case 0:  Otto.walk(2, 1000, 1); break;             // 前进2步
        case 1:  Otto.walk(2, 1000, -1); break;            // 后退2步
        case 2:  Otto.turn(2, 1000, 1); break;             // 左转2步
        case 3:  Otto.home(); delay(100); break;           // 回到初始位置
        case 4:  Otto.turn(2, 1000, -1); break;            // 右转2步
        case 5:  Otto.bend(1, 500, 1); break;              // 向左弯曲
        case 6:  Otto.bend(1, 2000, -1); break;            // 向右弯曲
        case 7:  Otto.shakeLeg(1, 1500, 1); break;         // 左腿摇晃
        case 8:  Otto.home(); delay(100); break;           // 回到初始位置
        case 9:  Otto.shakeLeg(1, 2000, -1); break;        // 右腿摇晃
        case 10: Otto.moonwalker(3, 1000, 25, 1); break;   // 左月球漫步
        case 11: Otto.moonwalker(3, 1000, 25, -1); break;  // 右月球漫步
        case 12: Otto.crusaito(2, 1000, 20, 1); break;     // 前进十字步
        case 13: Otto.crusaito(2, 1000, 20, -1); break;    // 后退十字步
        case 14: delay(100); break;                        // 短暂延迟
        case 15: Otto.flapping(2, 1000, 20, 1); break;     // 左拍打
        case 16: Otto.flapping(2, 1000, 20, -1); break;    // 右拍打
        case 17: delay(100); break;                        // 短暂延迟
        case 18: Otto.swing(2, 1000, 20); break;           // 摇摆
        case 19: Otto.tiptoeSwing(2, 1000, 20); break;     // 踮脚摇摆
        case 20: Otto.jitter(2, 1000, 20); break;          // 抖动
        case 21: Otto.updown(2, 1500, 20); break;          // 上下移动
        case 22: Otto.ascendingTurn(2, 1000, 50); break;   // 上升旋转
        case 23: Otto.jump(1, 2000); break;                // 跳跃
        case 24: delay(50); Otto.home(); break;            // 回到初始位置
        case 25: Otto.playGesture(OttoHappy); break;       // 开心手势
        case 26: Otto.playGesture(OttoSuperHappy); break;  // 超级开心
        case 27: Otto.playGesture(OttoSad); break;         // 悲伤
        case 28: Otto.playGesture(OttoVictory); break;     // 胜利
        case 29: Otto.playGesture(OttoAngry); break;       // 生气
        case 30: Otto.playGesture(OttoSleeping); break;    // 睡觉
        case 31: Otto.playGesture(OttoFretful); break;     // 焦虑
        case 32: Otto.playGesture(OttoLove); break;        // 爱心
        case 33: Otto.playGesture(OttoConfused); break;    // 困惑
        case 34: Otto.playGesture(OttoFart); break;        // 放屁
        case 35: Otto.playGesture(OttoWave); break;        // 挥手
        case 36: Otto.playGesture(OttoMagic); break;       // 魔法
        case 37: Otto.playGesture(OttoFail); break;        // 失败
        case 38: Otto.home(); break;                       // 回到初始位置
        case 39: currentState = MODE_COMPLETED; break;     // 完成所有动作
      }
      
      // 检查退出命令
      while(Serial.available() > 0)
      {
        processChar(Serial.read());
        if (currentMode == CMD_EXIT) {
          currentMode = -1;
          currentState = MODE_INIT;
          return;
        }
      }
      break;
      
    case MODE_COMPLETED:
      // 等待用户命令退出
      while(Serial.available() > 0)
      {
        processChar(Serial.read());
        if (currentMode == CMD_EXIT) {
          currentMode = -1;
          currentState = MODE_INIT;
          return;
        }
      }
      break;
      
    default:
      moveStep = 0;
      currentMode = -1;
      currentState = MODE_INIT;
      break;
  }
}




// 跟随功能（修改为非阻塞模式）
void runFollow()
{
  static bool initialized = false;
  
  switch(currentState) {
    case MODE_INIT:
      Otto.init(PIN_YL, PIN_YR, PIN_RL, PIN_RR, true, A6, PIN_Buzzer, PIN_Trigger, PIN_Echo);
      pinMode(PIN_ASSEMBLY, INPUT_PULLUP);
      Otto.sing(S_connection);
      Otto.home();
      delay(500);
      initialized = true;
      currentState = MODE_RUNNING;
      break;
      
    case MODE_RUNNING:
      if (digitalRead(PIN_ASSEMBLY) == LOW)
      {
        Otto.home();
        Otto.sing(S_happy_short);
        delay(500);
      }
      
      if (obstacleDetected)
      {
        Otto.walk(2, 500, 1);
      }
      
      delay(100);
      distance = Otto.getDistance();
      if (distance < 15)
      {
        obstacleDetected = true;
      }
      else
      {
        obstacleDetected = false;
        Otto.home();
      }
      
      // 检查退出命令
      while(Serial.available() > 0)
      {
        processChar(Serial.read());
        if (currentMode == CMD_EXIT) {
          initialized = false;
          currentMode = -1;
          currentState = MODE_INIT;
          return;
        }
      }
      break;
      
    default:
      initialized = false;
      currentMode = -1;
      currentState = MODE_INIT;
      break;
  }
}

// 运行功能（修改为非阻塞模式）
void runRun()
{
  static bool initialized = false;
  
  switch(currentState) {
    case MODE_INIT:
      Otto.init(PIN_YL, PIN_YR, PIN_RL, PIN_RR, true, A6, PIN_Buzzer, PIN_Trigger, PIN_Echo);
      pinMode(PIN_ASSEMBLY, INPUT_PULLUP);
      Otto.sing(S_connection);
      Otto.home();
      delay(500);
      initialized = true;
      currentState = MODE_RUNNING;
      break;
      
    case MODE_RUNNING:
      if (digitalRead(PIN_ASSEMBLY) == LOW)
      {
        Otto.home();
        Otto.sing(S_happy_short);
        delay(500);
      }
      
      if (!obstacleDetected)
      {
        Otto.walk(2, 500, 1);
      }
      
      delay(100);
      distance = Otto.getDistance();
      if (distance < 15)
      {
        obstacleDetected = true;
        Otto.home();
      }
      else
      {
        obstacleDetected = false;
      }
      
      // 检查退出命令
      while(Serial.available() > 0)
      {
        processChar(Serial.read());
        if (currentMode == CMD_EXIT) {
          initialized = false;
          currentMode = -1;
          currentState = MODE_INIT;
          return;
        }
      }
      break;
      
    default:
      initialized = false;
      currentMode = -1;
      currentState = MODE_INIT;
      break;
  }
}

void happyDance1()
{
  static bool initialized = false;
  static int danceStep = 0;
  static unsigned long stepStartTime = 0;
  static const unsigned long STEP_DURATION = 5000; // 每个舞蹈步骤持续时间
  static unsigned long lastArmUpdate = 0;         // 上次手臂更新时间[6](@ref)
  static const unsigned long ARM_UPDATE_INTERVAL = 50; // 手臂更新间隔(毫秒)
  
  switch(currentState) {
    case MODE_INIT:
      Otto.init(PIN_YL, PIN_YR, PIN_RL, PIN_RR, true, A6, PIN_Buzzer, PIN_Trigger, PIN_Echo);
      Otto.sing(S_connection);
      Otto.home();
      delay(50);
      danceStep = 0;
      stepStartTime = millis();
      initialized = true;
      
      // 初始化手臂振荡器参数[1](@ref)
      oscillators[4].SetO(0);         // 右臂偏移
      oscillators[4].SetA(50);          // 右臂幅度(30度)
      oscillators[4].SetT(300);         // 右臂周期(800ms)
      oscillators[4].SetPh(DEG2RAD(0)); // 右臂相位(0度)
      
      oscillators[5].SetO(0);           // 左臂偏移
      oscillators[5].SetA(50);          // 左臂幅度(30度)
      oscillators[5].SetT(300);         // 左臂周期(800ms)
      oscillators[5].SetPh(DEG2RAD(0)); // 左臂相位(180度，与右臂反相)[4](@ref)
      
      currentState = MODE_RUNNING;
      break;
      
    case MODE_RUNNING:
      unsigned long currentTime = millis();
      
      // 检查退出命令
      if (currentMode == CMD_EXIT) {
        initialized = false;
        currentMode = -1;
        currentState = MODE_INIT;
        Otto.home(); // 确保舵机归位
        return;
      }
      
      // 更新手臂位置（非阻塞方式）[6](@ref)
      if (currentTime - lastArmUpdate >= ARM_UPDATE_INTERVAL) {
        lastArmUpdate = currentTime;
        oscillators[4].refresh(); // 更新右臂位置
        oscillators[5].refresh(); // 更新左臂位置
      }
      
      // 检查是否需要执行下一步
      if (currentTime - stepStartTime >= STEP_DURATION) {
        danceStep++;
        stepStartTime = currentTime;
      }
      
      // 执行当前舞蹈步骤
      switch(danceStep) {
        case 0:  Otto.jitter(2, 750, 20); break;
        case 1:  Otto.crusaito(1, 800, 30, 1); break;
        case 2:  Otto.crusaito(1, 800, 30, -1); break;
        case 3:  Otto.crusaito(1, 800, 30, 1); delay(300); break;
        case 4:  Otto.walk(1, 1500, -1); break;
        case 5:  Otto.walk(2, 1000, 1); break;
        case 6:  Otto.moonwalker(1, 600, 30, 1); break;
        case 7:  Otto.moonwalker(1, 600, 30, -1); break;
        case 8:  Otto.moonwalker(1, 600, 30, 1); break;
        case 9:  Otto.moonwalker(1, 600, 30, -1); break;
        case 10: Otto.walk(1, 1500, -1); break;
        case 11: Otto.walk(2, 1000, 1); break;
        case 12: Otto.moonwalker(1, 600, 30, 1); break;
        case 13: Otto.moonwalker(1, 600, 30, -1); break;
        case 14: Otto.moonwalker(1, 600, 30, 1); break;
        case 15: Otto.moonwalker(1, 600, 30, -1); break;
        case 16: Otto.walk(1, 1500, -1); break;
        case 17: Otto.walk(2, 1000, 1); break;
        case 18: Otto.shakeLeg(1, 700, 1); break;
        case 19: Otto.shakeLeg(1, 700, -1); delay(1000); break;
        case 20: Otto.home(); break;
        case 21: Otto.moonwalker(1, 3000, 50, 1); break;
        case 22: Otto.moonwalker(1, 3000, 50, -1); delay(100); break;
        case 23: Otto.moonwalker(1, 3000, 50, -1); break;
        case 24: Otto.moonwalker(1, 3000, 50, 1); break;
        case 25: Otto.jump(1, 370); break;
        case 26: Otto.jump(1, 370); break;
        case 27: Otto.jump(1, 370); break;
        case 28: Otto.jump(1, 370); break;
        case 29: Otto.bend(1, 100, 1); break;
        case 30: Otto.bend(1, 100, -1); break;
        case 31: Otto.jump(1, 370); break;
        case 32: Otto.jump(1, 370); break;
        case 33: Otto.jump(1, 370); break;
        case 34: Otto.jump(1, 370); break;
        case 35: Otto.bend(1, 100, 1); break;
        case 36: Otto.bend(1, 100, -1); delay(500); break;
        case 37: Otto.crusaito(1, 800, 30, 1); break;
        case 38: Otto.crusaito(1, 800, 30, -1); break;
        case 39: Otto.crusaito(1, 800, 30, 1); delay(300); break;
        case 40: Otto.walk(1, 1500, -1); break;
        case 41: Otto.walk(2, 1000, 1); break;
        case 42: Otto.moonwalker(1, 600, 30, 1); break;
        case 43: Otto.moonwalker(1, 600, 30, -1); break;
        case 44: Otto.moonwalker(1, 600, 30, 1); break;
        case 45: Otto.moonwalker(1, 600, 30, -1); break;
        case 46: Otto.walk(1, 1500, -1); break;
        case 47: Otto.walk(2, 1000, 1); break;
        case 48: Otto.moonwalker(1, 600, 30, 1); break;
        case 49: Otto.moonwalker(1, 600, 30, -1); break;
        case 50: Otto.moonwalker(1, 600, 30, 1); break;
        case 51: Otto.moonwalker(1, 600, 30, -1); break;
        case 52: Otto.walk(1, 1500, -1); break;
        case 53: Otto.walk(2, 1000, 1); break;
        case 54: Otto.shakeLeg(1, 700, 1); break;
        case 55: Otto.shakeLeg(1, 700, -1); delay(1000); break;
        case 56: Otto.home(); break;
        case 57: Otto.moonwalker(1, 3000, 50, 1); break;
        case 58: Otto.moonwalker(1, 3000, 50, -1); delay(100); break;
        case 59: Otto.moonwalker(1, 3000, 50, -1); break;
        case 60: Otto.moonwalker(1, 3000, 50, 1); break;
        case 61: Otto.jump(1, 370); break;
        case 62: Otto.jump(1, 370); break;
        case 63: Otto.jump(1, 370); break;
        case 64: Otto.jump(1, 370); break;
        case 65: Otto.bend(1, 100, 1); break;
        case 66: Otto.bend(1, 100, -1); break;
        case 67: Otto.jump(1, 370); break;
        case 68: Otto.jump(1, 370); break;
        case 69: Otto.jump(1, 370); break;
        case 70: Otto.jump(1, 370); break;
        case 71: Otto.bend(1, 100, 1); break;
        case 72: Otto.bend(1, 100, -1); break;
        case 73: Otto.updown(3, 900, 30); break;
        case 74: Otto.jitter(3, 1000, 20); break;
        case 75: Otto.updown(3, 900, 30); break;
        case 76: Otto.jitter(3, 1000, 20); break;
        case 77: Otto.jump(1, 400); break;
        case 78: Otto.jump(1, 400); break;
        case 79: Otto.jump(1, 400); break;
        case 80: Otto.jump(1, 400); break;
        case 81: Otto.jitter(4, 800, 20); break;
        case 82: Otto.jump(1, 400); break;
        case 83: Otto.jump(1, 400); break;
        case 84: Otto.jump(1, 400); break;
        case 85: Otto.jump(1, 400); break;
        case 86: Otto.jitter(2, 800, 20); delay(1500); break;
        case 87: Otto.home(); break;
        case 88: Otto.moonwalker(1, 3000, 50, 1); break;
        case 89: Otto.moonwalker(1, 3000, 50, -1); delay(100); break;
        case 90: Otto.moonwalker(1, 3000, 50, -1); break;
        case 91: Otto.moonwalker(1, 3000, 50, 1); break;
        case 92: Otto.jump(1, 370); break;
        case 93: Otto.jump(1, 370); break;
        case 94: Otto.jump(1, 370); break;
        case 95: Otto.jump(1, 370); break;
        case 96: Otto.bend(1, 100, 1); break;
        case 97: Otto.bend(1, 100, -1); break;
        case 98: Otto.jump(1, 370); break;
        case 99: Otto.jump(1, 370); break;
        case 100: Otto.jump(1, 370); break;
        case 101: Otto.jump(1, 370); break;
        case 102: Otto.bend(1, 100, 1); break;
        case 103: Otto.bend(1, 100, -1); break;
        case 104: Otto.moonwalker(1, 3000, 50, 1); break;
        case 105: Otto.moonwalker(1, 3000, 50, -1); delay(100); break;
        case 106: Otto.moonwalker(1, 3000, 50, -1); break;
        case 107: Otto.moonwalker(1, 3000, 50, 1); break;
        case 108: Otto.jump(1, 370); break;
        case 109: Otto.jump(1, 370); break;
        case 110: Otto.jump(1, 370); break;
        case 111: Otto.jump(1, 370); break;
        case 112: Otto.bend(1, 100, 1); break;
        case 113: Otto.bend(1, 100, -1); break;
        case 114: Otto.jump(1, 370); break;
        case 115: Otto.jump(1, 370); break;
        case 116: Otto.jump(1, 370); break;
        case 117: Otto.jump(1, 370); break;
        case 118: Otto.bend(1, 100, 1); break;
        case 119: Otto.bend(1, 100, -1); break;
        case 120: Otto.home(); delay(10000); break;
        case 121: currentState = MODE_COMPLETED; break;
      }
      break;
      
    case MODE_COMPLETED:
      // 等待用户命令退出
      if (currentMode == CMD_EXIT) {
        initialized = false;
        currentMode = -1;
        currentState = MODE_INIT;
        return;
      }
      break;
      
    default:
      initialized = false;
      danceStep = 0;
      currentMode = -1;
      currentState = MODE_INIT;
      break;
  }
}

// 取回功能（修改为非阻塞模式）
void runRetrieve()
{
  static bool initialized = false;
  
  switch(currentState) {
    case MODE_INIT:
      Otto.init(PIN_YL, PIN_YR, PIN_RL, PIN_RR, true, A6, PIN_Buzzer, PIN_Trigger, PIN_Echo);
      pinMode(PIN_ASSEMBLY, INPUT_PULLUP);
      Otto.sing(S_connection);
      Otto.home();
      delay(500);
      initialized = true;
      currentState = MODE_RUNNING;
      break;
      
    case MODE_RUNNING:
      if (digitalRead(PIN_ASSEMBLY) == LOW)
      {
        Otto.home();
        Otto.sing(S_happy_short);
        delay(500);
      }
      
      if (obstacleDetected)
      {
        Otto.walk(1, 1000, 1);
      }
      else
      {
        Otto.home();
      }
      
      distance = Otto.getDistance();
      if (distance < 15)
      {
        obstacleDetected = true;
      }
      else
      {
        obstacleDetected = false;
      }
      
      // 检查退出命令
      while(Serial.available() > 0)
      {
        processChar(Serial.read());
        if (currentMode == CMD_EXIT) {
          initialized = false;
          currentMode = -1;
          currentState = MODE_INIT;
          return;
        }
      }
      break;
      
    default:
      initialized = false;
      currentMode = -1;
      currentState = MODE_INIT;
      break;
  }
}

// 测量距离（修改为非阻塞模式）
void runMeasureDistance()
{
  static bool initialized = false;
  static unsigned long lastMeasure = 0;
  static const unsigned long MEASURE_INTERVAL = 2000; // 测量间隔（毫秒）
  
  switch(currentState) {
    case MODE_INIT:
      Otto.init(PIN_YL, PIN_YR, PIN_RL, PIN_RR, true, A6, PIN_Buzzer, PIN_Trigger, PIN_Echo);
      initialized = true;
      lastMeasure = millis();
      currentState = MODE_RUNNING;
      break;
      
    case MODE_RUNNING:
      unsigned long currentTime = millis();
      
      if (currentTime - lastMeasure >= MEASURE_INTERVAL) {
        distance = Otto.getDistance();
        Serial.print("Distance: ");
        Serial.println(distance);
        lastMeasure = currentTime;
      }
      
      // 检查退出命令
      while(Serial.available() > 0)
      {
        processChar(Serial.read());
        if (currentMode == CMD_EXIT) {
          initialized = false;
          currentMode = -1;
          currentState = MODE_INIT;
          return;
        }
      }
      break;
      
    default:
      initialized = false;
      currentMode = -1;
      currentState = MODE_INIT;
      break;
  }
}


void setup() {
    Serial.begin(57600);
    Otto.init(PIN_YL, PIN_YR, PIN_RL, PIN_RR, true, A6, PIN_Buzzer, PIN_Trigger, PIN_Echo); // 初始化时使用正确的引脚

    // 初始化舵机
    setupServos();
    printHelp();
    // 初始化MAX7219点阵模块
    lc.shutdown(0, false);  // 唤醒芯片（0=第1个模块）
    lc.setIntensity(0, 8);  // 设置亮度（0-15，8为中等亮度）
    lc.clearDisplay(0);     // 清除屏幕
    showSmiley();           // 显示笑脸表情
    
}
void loop() {
    // 处理串口输入
    while (Serial.available() > 0) {
        processChar(Serial.read());
    }

    // 执行当前选择的功能
    if (currentMode >= 1 && currentMode <= 15) {
        switch (currentMode) {
            case CMD_CALIBRATION:
                runCalibration();
                break;
            case CMD_MOVE_FORWARD:  // 使用新的前进函数
                runMoveForward();
                break;
            case CMD_AVOID_OBSTACLE:
                runAvoidObstacle();
                break;
            case CMD_MOVE_BACKWARD:
                runMoveBackward();
                break;
            case CMD_HAPPY_DANCE:
                happyDance1();
                break;
            case CMD_ALL_MOVES:
                runAllMoves();
                break;
            case CMD_FOLLOW:
                runFollow();
                break;
            case CMD_RUN:
                runRun();
                break;
            case CMD_RETRIEVE:
                runRetrieve();
                break;
            case CMD_MEASURE_DISTANCE:
                runMeasureDistance();
                break;
            default:
                currentMode = -1;
                break;
        }
    } else if (currentMode == CMD_HELP) {
        printHelp();
        currentMode = -1;
    }
  
    // 短暂延时，避免CPU占用过高
    delay(10);
}