// ======================= KHAI BÁO THƯ VIỆN =======================
#include <Wire.h>                     // Thư viện kết nối I2C cho ESP32 tới các thiết bị
#include <Adafruit_MCP23X17.h>        // Khai báo thư viện dùng MCP23017
#include "TCA9548.h"                  // Thư viện PCA9548A
#include <LCDI2C_Multilingual.h>      // Thư viện LCD2004 có chứa các ngôn ngữ khác nhau
#include <virtuabotixRTC.h>           // Thư viện module CLOCK Real-Time Clock DS1302
#include <Adafruit_Sensor.h>          // Thư viện hỗ trợ cảm biến các loại
#include <Adafruit_MPU6050.h>         // Thư viện cho cảm biến gia tốc MPU6050
#include <Adafruit_LSM303_U.h>        // Thư viện cho cảm biến gia tốc và trọng trường GY511
#include <Kalman.h>                   // Thư viện lọc nhiễu Kalman
#include <Adafruit_PWMServoDriver.h>  // Thư viện dành cho điều khiển Servo 12bit sử dụng module PCA9685

// ======================= ĐẶT ĐỊA CHỈ I2C CHO TỪNG THIẾT BỊ =======================
// Sử dụng IC Multiplex I2C [PCA9548A] ==> Dùng để kết nối được thêm nhiều cảm biến hoặc module có sử dụng giao thức I2C
#define CHANNEL_LCD2004 3     // Đặt địa chỉ I2C cho LCD tại chân 0
#define CHANNEL_MCP23017 4    // Đặt địa chỉ I2C cho MCP tại chân 1
#define CHANNEL_MPU6050 2     // Đặt địa chỉ I2C cho MPU tại chân 3
#define CHANNEL_GY511 0       // Đặt địa chỉ I2C cho GY511 tại chân 4
#define CHANNEL_PCA9685 6     // Đặt địa chỉ I2C cho PCA9685 tại chân 5 SERVO

// ======================= TỐC ĐỘ GIỚI HẠN CHO ĐỘNG CƠ HOẠT ĐỘNG =======================
// Sử dụng kĩ thuật [PWM] để điều khiển tốc độ động cơ
#define MAX_SPEED 255     // Tốc độ cao nhất
#define MIN_SPEED 0       // Tốc độ thấp nhất

// ======================= KHAI BÁO CHÂN ĐIỀU KHIỂN ĐỘNG CƠ =======================
// Điều khiển động cơ thông qua module [L298N]
#define LEFT_A 32
#define LEFT_B 33
#define RIGHT_A 25
#define RIGHT_B 26
#define EN_LEFT 27
#define EN_RIGHT 14

// ======================= TỐC ĐỘ GIỚI HẠN CHO ĐỘNG CƠ SERVO =======================
// Giá trị min & max tính theo xung PWM (tùy loại servo)
#define SERVOMIN  110  // Dải xung thấp nhất
#define SERVOMAX  490  // Dải xung rộng nhất

// ======================= KHÁC =======================
#define NUM_ENCODERS 4   // Số lượng encoders đang sử dụng
#define WHEEL_RADIUS 3.3 // Bán kính bánh xe

// ======================= ĐẶT TÊN RÚT GỌN CHO CÁC THƯ VIỆN SỬ DỤNG - SHORT NAMED LIB =======================
Adafruit_MCP23X17 mcp;                                                        // Thư viện MCP23017 
TCA9548 tca(0x70);                                                            // Thư viện PCA9548A
LCDI2C_Generic lcd(0x27, 20, 4);                                              // Thư viện LCD2004
virtuabotixRTC myRTC(23, 27, 14);                                             // Thư viện DS1302
Adafruit_MPU6050 mpu;                                                         // Thư viện MPU6050
Kalman kalmanPitch, kalmanRoll, kalmanYaw;                                    // Thư viện Kalman lọc nhiễu [Pitch, Roll, Yaw]
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);   // Thư viện GY511 [Gia tốc - Accelartion]
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);         // Thư viện GY511 [Từ kế - Magnetometer]
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();                      // Thư viện Servo [PWM] 


// ======================= KHAI BÁO BIẾN TOÀN CỤC - GLOBAL VARIABLES =======================
// LINE IR
int lineState[5] = {0};

// GY511
float angleGYX, angleGYY, angleGYZ;

// MPU6050
float accX, accY, accZ;             // Biến GIA TỐC [Accelaration] dành cho 3 trục X,Y,Z
float gyroX, gyroY, gyroZ;          // Biến TỪ KẾ [Magnetometer] dành cho 3 trục X,Y,Z
float angleX, angleY;               // Biến GÓC QUAY [Angle] dành cho 2 trục X,Y
unsigned long timer;                // Biến THỜI GIAN dành cho tính toán và qui đổi

// Thuật toán lọc nhiễu Kalman [Kalman Filter Algorithm]
float lastUpdate = 0;               // Biến phụ cho thuật toán lọc Kalman
float angleGYZFiltered = 0;         // Lọc Kalman cho trục Z [Yaw]

// Biến phụ khác
int speed = 230;                    // Tốc độ của động cơ qui về phương pháp PWM chứ không phải km/h hay m/s
float Pwheel = 2*PI*WHEEL_RADIUS;   // Chu vi bánh xe <=> chu vi hình tròn
float targetYaw = 0.0;
float startYaw = 0.0;
bool YawInitSave = false;
bool hasSetTargetYaw = false;
bool stop_flag = false;
int MODE_STAGE = 0; // 0: đi Y, 1: quay 90, 2: đi X

// Biến đọc tín hiệu Encoders
const uint8_t encoderPins_MCP[4] = {12, 13, 14, 15};                  // Sử dụng các GPIOs của MCP23017

// Ngắt ngoài + Encoders
const uint8_t encoderPins_GPIO[4] = {36, 39, 34, 19};                 // Sử dụng các GPIOs của ESP32
volatile unsigned long lastDebounceTime[NUM_ENCODERS] = {0};          // Biến lưu giá trị tác động gần nhất của từng Encoders <=> chống dội
volatile int encoderCounts[NUM_ENCODERS] = {0};                       // Biến đềm số lượng xung của từng Encoders
volatile uint8_t lastEncoderState[NUM_ENCODERS] = {0};                // Biến lưu trạng thái xung của từng Encoders
const unsigned long DEBOUNCE_DELAY = 2000;                            // Biến thời gian tạo độ trễ cho các lần tác động của từng Encoders
const int pulse_def = 40;                                             // Số xung <=> số lỗ trên vòng Encoders
float rotate_per_eva[NUM_ENCODERS] = {0};                             // Biến lưu giá trị số vòng đã quay được của động cơ


// ======================= CÁC HÀM KIỂM TRA CƠ BẢN - BASIC TEST FUNCTION =======================
void TEST_LINE();                                                                 // Hàm kiểm tra cảm biến dò line              ==> Cảm biến IR
void TEST_L298N_VER1();                                                           // Hàm kiểm tra L298N + MCP23017              ==> Xung dạng Digital [Max,Min]
void TEST_L298N_VER2(int speed);                                                  // Hàm kiểm tra L298N + ESP32                 ==> Xung dạng Analog  [Min,...,Max]
void TEST_DS1302();                                                               // Hàm kiểm tra DS1302                        ==> Module thời gian thực
void TEST_MPU6050_GY511_KALMAN();                                                 // Hàm kiểm tra MPU6050 + GY511 + Kalman
void TEST_LCD2004();                                                              // Hàm kiểm tra hiển thị màn hình LCD20x4
// Hàm kiểm tra ENCODER
void TEST_ENCODER_VER1();                                                         // Hàm kiểm tra Encoder + MCP23017 + PCA9548A
void TEST_ENCODER_VER2();                                                         // Hàm kiểm tra Encoder + ESP32 + Không ngắt ngoài
void TEST_ENCODER_VER3();                                                         // Hàm kiểm tra Encoder + ESP32 + Ngắt ngoài
void TEST_SERVO(int F_Ser, int L_Ser, int Step, int angle);                       // Hàm kiểm tra PCA9685                       ==> Module điều khiển Servo
void TEST_GY511();                                                                // Hàm kiểm tra GY511                         ==> Module gia tốc và gia tốc kế
void TEST_MPU6050();                                                              // Hàm kiểm tra MPU6050                       ==> Module gia tốc 
// Hàm kiểm tra L298N + ENCODER
void TEST_L298N_ENCODER_VER1(int speed);                                           // ENCODER + MCP23017
void TEST_L298N_ENCODER_VER2(int speed);                                           // ENCODER + GPIO
void TEST_L298N_ENCODER_VER3(int speed);                                           // Encoder + GPIO + Ngắt ngoài
// Hàm kiểm tra di chuyển theo mong muốn [Khoảng cách, Góc quay, Tọa độ]
bool TEST_DESIRED_ROTATION(float DesWheelRotate);                                  // Hàm kiểm tra số vòng quay bánh xe mong muốn
bool TEST_DESIRED_DISTANCE(float des_distance);                                    // Hàm kiểm tra xe đi tới khoảng cách mong muốn
// max_angle - Góc quay lớn nhất [0] - Bị giới hạn từ 0-360 độ [1] - Xoay bất kì góc nào 
bool TEST_DESIRED_ANGLE(float des_angle, int max_angle);                           // Hàm kiểm tra xe đi xoay 1 góc mong muốn

// ======================= CÁC HÀM KHỞI TẠO - INITIALIZE FUNCION =======================
void INIT_LCD();                                            // Khởi tạo màn hình LCD2004
void INIT_PCA();                                            // Khởi tạo module PCA9548A
void INIT_MCP();                                            // Khởi tạo module MCP23017
void INIT_MPU();                                            // Khởi tạo module MPU6050
void INIT_GY511();                                          // Khởi tạo module GY511
void INIT_SERVO();                                          // Khởi tạo module PCA9685
void INIT_LINE();                                           // Khởi tạo các biến đọc IR
void INIT_L298N();                                          // Khởi tạo module L298N
// readMode       - Đọc ENCODER thông qua                    [0] - MCP23017 + PCA9548A hoặc [1] - ESP32
// interruptMode  - Cần ngắt ngoài khi đọc bằng ESP32 không? [0] - Không               hoặc [1] - Có
void INIT_ENCODER(int readMode, bool interruptMode);        // Khởi tạo module ENCODER
void INIT_DS1302();                                         // Khởi tạo module Timer DS1302

// ======================= CÁC HÀM ĐIỀU KHIỂN - CONTROL FUNCTION =======================
void CON_SERVO(int F_Ser, int L_Ser, int Step, int angle);  // Hàm dùng để vừa chạy động cơ và đọc xung encoder
// Điều chỉnh hướng đi của xe
void SPIN_LEFT(int speed);                                  // Hướng trái       <=> Quay qua trái tại chỗ
void SPIN_RIGHT(int speed);                                 // Hướng phải       <=> Quay qua phải tại chỗ
void GO_FORWARD(int speed);                                 // Hướng tiến thẳng 
void GO_BACKWARD(int speed);                                // Hướng lùi thẳng
void STOP_ALL();                                            // Dừng hết
void LEFT_MOTOR_FORWARD(int speed);
void LEFT_MOTOR_BACKWARD(int speed);
void RIGHT_MOTOR_FORWARD(int speed);
void RIGHT_MOTOR_BACKWARD(int speed);


// ======================= CÁC HÀM ĐỌC CẢM BIẾN - READ FUNCTION =======================
void READ_MPU6050();                                        // Đọc cảm biến MPU6050
void READ_GY511();                                          // Đọc cảm biến GY511
void READ_ENCODER(int Sel);                                 // Đọc cảm biến ENCODER
void READ_LINE();                                           // Đọc cảm biến dò LINE
void READ_DS1302();                                         // Đọc cảm biến DS1302 - Timer

// ======================= CÁC CHẾ ĐỘ ĐIỀU KHIỂN CƠ BẢN - BASIC CONTROL FUNCTION =======================
// Distance            - Khoảng cách cần di chuyển
void MODE_1(float Distance);                      // Chế độ 1 <=> Điều khiển xe tới khoảng cách mong muốn
// DesWheelRotate      - Số vòng quay bánh xe mong muốn
void MODE_2(float DesWheelRotate);              // Chế độ 2 <=> Điều khiển xe với số vòng quay bánh xe mong muốn
// DesAngle            - Góc quay mong muốn
void MODE_3(float DesAngle);                    // Chế độ 3 <=> Điều khiển xe tới góc quay mong muốn trong phạm vi 0-360 độ [VD: Tại góc 40 độ thì chỉ cần quay 320 độ]
// DesAngle            - Góc quay mong muốn
void MODE_4(float DesAngle);                    // Chế độ 4 <=> Điều khiển xe tới góc quay mong muốn trong phạm vi 0-360 độ [VD: Tại góc 40 độ thì quay đúng 360 độ]
// DesAngle            - Góc quay mong muốn
// DesDistance         - Quãng đường đi mong muốn
void MODE_5(float DesDistance, float DesAngle); // Chế độ 5 <=> Điều khiển xe tới quãng đường và quẹo 1 góc mong muốn SO với góc HIỆN TẠI
// DesAngle            - Góc quay mong muốn
// DesDistance         - Quãng đường đi mong muốn
void MODE_6(float DesDistance, float DesAngle); // Chế độ 6 <=> Điều khiển xe tới quãng đường và quẹo 1 góc mong muốn với MỌI góc HIỆN TẠI
// (xCar, yCar) - Tọa độ xe hiện tại
// (xMark, yMark) - Tọa độ điểm đến
void MODE_7(int xCar, int yCar, int xMark, int yMark);      // Chế độ 7 <=> Nhập tọa độ để xe đi tới theo CẠNH GÓC VUÔNG

// ======================= CÁC HÀM KHÁC - OTHERs FUNCTION =======================
uint16_t angleToPulse(int angle);                             // Hàm trả về góc quay mong muốn cho nhiệm vụ điều khiển Servo

// index               - Vị trí Encoder mà đang sử dụng chức năng ngắt ngoài trên ESP32
void IRAM_ATTR handleEncoderInterrupt(uint8_t index)          // Hàm xử lí ngắt ngoài + chống dội 
{
  /* === Giải thích nguyên lí ===
  now               - Giá trị thời gian hiện tại [tính từ lúc chạy chương trình]
  lastDebounceTime  - Thời gian gần nhất mà đọc được xung tín hiệu từ Encoder hợp lệ
  DEBOUNCE_DELAY    - Độ trễ cần thiết để tránh việc bị nhiễu tín hiệu hoặc bị dội liên tục
  */
  unsigned long now = micros();                             
  if (now - lastDebounceTime[index] > DEBOUNCE_DELAY) {     
    encoderCounts[index]++;
    lastDebounceTime[index] = now;
  }
}
// Hàm xử lý ngắt với 4 chân ngắt dành cho 4 Encoders
void IRAM_ATTR encoderISR0() { handleEncoderInterrupt(0); }   // Ngắt ngoài tại vị trí Encoder thứ 0
void IRAM_ATTR encoderISR1() { handleEncoderInterrupt(1); }   // Ngắt ngoài tại vị trí Encoder thứ 1
void IRAM_ATTR encoderISR2() { handleEncoderInterrupt(2); }   // Ngắt ngoài tại vị trí Encoder thứ 2
void IRAM_ATTR encoderISR3() { handleEncoderInterrupt(3); }   // Ngắt ngoài tại vị trí Encoder thứ 3

void setup() {
  Serial.begin(115200);                                       // Khởi tạo Serial Monitor ==> Trực quan hóa tính toán, xử lí và điều khiển
  Wire.begin(21, 22);                                         // Khởi tạo giao thức I2C trên ESP32 ==> [21 - SDA] và [22 - SCL]
  Wire.setClock(400000);                                      // Tốc độ đọc TỐI ĐA của I2C [100Khz, 1MHz, 400KHz]

  // Khai báo module cần sử dụng 
  // INIT_DS1302();
  INIT_LCD();
  INIT_PCA();
  INIT_MCP();  
  INIT_MPU();
  INIT_GY511();
  // INIT_SERVO();
  INIT_LINE();
  INIT_L298N();  
  INIT_ENCODER(1,1);
  delay(7000);
}

void loop() {
  // TEST_LINE();
  // TEST_DS1302();
  // TEST_LCD2004();

  // TEST_ENCODER_VER1();
  // TEST_ENCODER_VER2();
  // TEST_ENCODER_VER3();

  // TEST_L298N_VER1(); // L298N + MCP23017 ==> Nominal Speed [0,1]
  // TEST_L298N_VER2(speed); // L298N + ESP32 ==> Adjustable Speed [PWM]

  // TEST_SERVO(0, 5, 1, 180);

  // TEST_MPU6050_GY511_KALMAN();
  // TEST_GY511();
  // TEST_MPU6050();

  // TEST_L298N_ENCODER_VER1(speed);
  // TEST_L298N_ENCODER_VER2(speed);
  // TEST_L298N_ENCODER_VER3(speed);
  
  // MODE_1(50.0);
  // MODE_2(4.0);
  // MODE_3(90.0);
  // MODE_4(180.0);
  // MODE_5(100.0, 90.0);
  // MODE_6(100.0, 90.0);
  MODE_7(0, 0, 5, 5);
}

// ================Khai báo các hàm thông số================
uint16_t angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// ======================= CÁC HÀM ĐỌC CẢM BIẾN - READ FUNCTION =======================
void READ_MPU6050()
{
  // === MPU6050 ===
  tca.selectChannel(CHANNEL_MPU6050); delay(10);
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  float dt = (float)(micros() - timer) / 1000000.0;
  timer = micros();

  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  gyroX = g.gyro.x * 180 / PI;
  gyroY = g.gyro.y * 180 / PI;

  float accAngleX = atan2(accY, accZ) * 180 / PI;
  float accAngleY = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;

  // === Kalman ===
  angleX = kalmanPitch.getAngle(accAngleX, gyroX, dt);
  angleY = kalmanRoll.getAngle(accAngleY, gyroY, dt);
}

void READ_GY511()
{
  // === Đọc dữ liệu từ GY-511 ===
  tca.selectChannel(CHANNEL_GY511); delay(10);
  sensors_event_t accEvent, magEvent;
  accel.getEvent(&accEvent);
  mag.getEvent(&magEvent);

  // === Tính góc nghiêng (X, Y) từ accelerometer ===
  angleGYX = atan2(accEvent.acceleration.y, accEvent.acceleration.z) * 180 / PI; // Tính góc quay Pitch
  angleGYY = atan2(accEvent.acceleration.x, accEvent.acceleration.z) * 180 / PI; // Tính góc quay Roll
  angleGYZ = atan2(magEvent.magnetic.y, magEvent.magnetic.x) * 180 / PI; // Tính góc quay Yaw
  if (angleGYZ < 0) angleGYZ += 360;  // Đảm bảo giá trị dương từ 0 - 360 độ

  // Tính thời gian trôi (delta t)
  float now = millis();
  float dt = (now - lastUpdate) / 1000.0;
  lastUpdate = now;

  // Giả sử không có tốc độ góc (gyroZ = 0)
  angleGYZFiltered = kalmanYaw.getAngle(angleGYZ, 0.0, dt);
  delay(200);
}

void READ_ENCODER(int Sel)
{
  switch(Sel)
  {
    case 1:
      for (uint8_t i = 0; i < 4; i++) 
      {
        tca.selectChannel(CHANNEL_MCP23017);
        uint8_t currentState = mcp.digitalRead(encoderPins_MCP[i]);
        if (lastEncoderState[i] == LOW && currentState == HIGH) 
        {
          encoderCounts[i]++; // Đếm khi có cạnh lên
        }
        lastEncoderState[i] = currentState;
        rotate_per_eva[i] = encoderCounts[i] / pulse_def;
      }
    break;

    case 2:
      for (uint8_t i = 0; i < 4; i++) 
      {
        uint8_t currentState = digitalRead(encoderPins_GPIO[i]);
        if (lastEncoderState[i] == LOW && currentState == HIGH) 
        {
          encoderCounts[i]++; // Đếm khi có cạnh lên
        }
        lastEncoderState[i] = currentState;
        rotate_per_eva[i] = encoderCounts[i] / pulse_def;
      }
    break;

    case 3:
      for (uint8_t i = 0; i < NUM_ENCODERS; i++) 
      {
        rotate_per_eva[i] = encoderCounts[i] / pulse_def;
      }
    break;
  }
  
  
}

void READ_LINE()
{
  tca.selectChannel(CHANNEL_MCP23017); delay(10);
  for (uint8_t i = 0; i < 5; i++) 
  {
    lineState[i] = mcp.digitalRead(i); // Đọc trạng thái cảm biến IR      
  }
}

void READ_DS1302()
{
  myRTC.updateTime();

  tca.selectChannel(CHANNEL_LCD2004); delay(10);
  lcd.clear();
  // In trên hàng thứ 1 từ trên xuống
  lcd.setCursor(0, 0); 
  lcd.print("TEST DATE TIME");

  // In trên hàng thứ 2 từ trên xuống
  lcd.setCursor(0, 1); 
  if (myRTC.dayofmonth < 10) lcd.print("0"); lcd.print(myRTC.dayofmonth);
  lcd.setCursor(2, 1);  
  lcd.print("/"); 
  if (myRTC.month < 10) lcd.print("0"); lcd.print(myRTC.month);
  lcd.setCursor(5, 1); 
  lcd.print("/"); 
  lcd.print(myRTC.year);
  lcd.setCursor(10, 1);
  lcd.print("-"); 
  if (myRTC.hours < 10) lcd.print("0"); lcd.print(myRTC.hours);
  lcd.setCursor(13, 1); 
  lcd.print(":"); 
  if (myRTC.minutes < 10) lcd.print("0"); lcd.print(myRTC.minutes);
  lcd.setCursor(16, 1); 
  lcd.print(":"); 
  if (myRTC.seconds < 10) lcd.print("0");lcd.print(myRTC.seconds);       
 
  delay(1000);
}

// ================Khai báo các chế độ điều khiển - Control Mode================
void MODE_1(float Distance)
{
  TEST_DESIRED_DISTANCE(Distance);
}

void MODE_2(float DesWheelRotate)
{
  TEST_DESIRED_ROTATION(DesWheelRotate);
}

void MODE_3(float DesAngle)
{
  TEST_DESIRED_ANGLE(DesAngle, 0);
}

void MODE_4(float DesAngle)
{
  TEST_DESIRED_ANGLE(DesAngle, 1);
}

void MODE_5(float DesDistance, float DesAngle) 
{
  // Chọn kênh LCD
  tca.selectChannel(CHANNEL_LCD2004);
  delay(10);

  lcd.setCursor(0, 0);
  lcd.printf("MODE 5 - %.0fcm-%.0f*", DesDistance, DesAngle);

  switch (MODE_STAGE)
  {
    case 0: // Giai đoạn 1: Đi thẳng
      lcd.setCursor(0, 1);
      lcd.printf("      FORWARD    ");
      TEST_DESIRED_DISTANCE(DesDistance);
      break;

    case 1: // Giai đoạn 2: Quay 1 góc mong muốn
      lcd.setCursor(0, 1);
      lcd.printf("       %.2f deg    ", DesAngle);
      TEST_DESIRED_ANGLE(DesAngle, 0);
      break;

    default: // Giai đoạn 3: Kết thúc
      lcd.setCursor(0, 1);
      lcd.print("    FINISHED     ");
      break;
  }
}

void MODE_6(float DesDistance, float DesAngle) 
{
  // Chọn kênh LCD
  tca.selectChannel(CHANNEL_LCD2004);
  delay(10);

  lcd.setCursor(0, 0);
  lcd.printf("MODE 6 - %.0fcm-%.0f*", DesDistance, DesAngle);

  switch (MODE_STAGE)
  {
    case 0: // Giai đoạn 1: Đi thẳng
      lcd.setCursor(0, 1);
      lcd.printf("     FORWARD     ");
      TEST_DESIRED_DISTANCE(DesDistance);
      break;

    case 1: // Giai đoạn 2: Quay 1 góc mong muốn
      lcd.setCursor(0, 1);
      lcd.printf("       %.2f deg    ", DesAngle);
      TEST_DESIRED_ANGLE(DesAngle, 1);
      break;

    default: // Giai đoạn 3: Kết thúc
      lcd.setCursor(0, 1);
      lcd.print("    FINISHED     ");
      break;
  }
}

void MODE_7(int xCar, int yCar, int xMark, int yMark)
{
  float dx = (float)(xMark - xCar); 
  float dy = (float)(yMark - yCar);

  // Serial.printf("Dx: %.1f - Dy: %.1f\n",dx, dy);
  // Chọn kênh LCD
  tca.selectChannel(CHANNEL_LCD2004);
  delay(10);

  lcd.setCursor(0, 0);
  lcd.print("       MODE 7      ");
  lcd.setCursor(0, 2);
  lcd.printf("From (%d,%d) To (%d,%d)", xCar, yCar, xMark, yMark);

  switch (MODE_STAGE)
  {
    case 0: // Giai đoạn 1: Đi theo Y
      lcd.setCursor(0, 1);
      lcd.printf("        DY       ");
      if (TEST_DESIRED_ROTATION(dy)) MODE_STAGE++;
      break;

    case 1: // Giai đoạn 2: Quay 90 độ
      lcd.setCursor(0, 1);
      lcd.printf("       90 deg    ");
      if (TEST_DESIRED_ANGLE(90.0, 0)) MODE_STAGE++;
      break;

    case 2: // Giai đoạn 3: Đi theo X
      lcd.setCursor(0, 1);
      lcd.printf("        DX       ");
      if (TEST_DESIRED_ROTATION(dy)) MODE_STAGE++;
      break;

    default:
      lcd.setCursor(0, 1);
      lcd.print("    FINISHED     ");
      STOP_ALL();
      break;
  }
}

// ================Khai báo các hàm khởi tạo - Initialize Function================
void INIT_DS1302()
{
  myRTC.setDS1302Time(23, 52, 10, 2, 5, 2, 2025);
}

void INIT_ENCODER(int readMode, bool interruptMode)
{
  if(readMode == 0) // ============== ENCODER sử dụng MCP23017 ========================
  { 
    for(uint8_t i = 0; i <= 4; i++) // Lưu dưới dạng mảng thay vì từng biến
    {
      mcp.pinMode(encoderPins_MCP[i], INPUT_PULLUP);
      lastEncoderState[i] = mcp.digitalRead(encoderPins_MCP[i]); // Ghi nhận trạng thái ban đầu
    }
  }

  else if(readMode == 1) // ============== ENCODER sử dụng GPIO ESP32 - thay vì MCP ========================
  {
    for(uint8_t i = 0; i <= 4; i++) // Lưu dưới dạng mảng thay vì từng biến
    {
      pinMode(encoderPins_GPIO[i], INPUT_PULLUP);
      lastEncoderState[i] = digitalRead(encoderPins_GPIO[i]); // Ghi nhận trạng thái ban đầu
    }
    if(interruptMode) // ============== ENCODER sử dụng ngắt ngoài ============== 
    {  
      // Sử dụng ngắt ngoài encoder
      attachInterrupt(digitalPinToInterrupt(encoderPins_GPIO[0]), encoderISR0, FALLING);
      attachInterrupt(digitalPinToInterrupt(encoderPins_GPIO[1]), encoderISR1, FALLING);
      attachInterrupt(digitalPinToInterrupt(encoderPins_GPIO[2]), encoderISR2, FALLING);
      attachInterrupt(digitalPinToInterrupt(encoderPins_GPIO[3]), encoderISR3, FALLING);
    }
  }
  else Serial.print("Encoder không có readMode: "); Serial.print(readMode);
}

void INIT_L298N()
{
  // =================L298N sử dụng MCP23017=================
  // for(uint8_t pin_MOTOR = 8; pin_MOTOR <= 11; pin_MOTOR++) // Sử dụng cụm INTB
  // {
  //   tca.selectChannel(CHANNEL_MCP23017); delay(10);
  //   mcp.pinMode(pin_MOTOR, OUTPUT);
  //   mcp.digitalWrite(pin_MOTOR, LOW); // Khởi tạo tắt động cơ
  // }

  // =================L298N sử dụng ESP32=================
  pinMode(LEFT_A, OUTPUT);
  pinMode(LEFT_B, OUTPUT);
  pinMode(RIGHT_A, OUTPUT);
  pinMode(RIGHT_B, OUTPUT);
  pinMode(EN_LEFT, OUTPUT);
  pinMode(EN_RIGHT, OUTPUT);
}

void INIT_LINE()
{
  // Khai báo INPUT 5 cảm biến IR dò line
  for(uint8_t pin_IR = 0; pin_IR <= 4; pin_IR++)
  {
    mcp.pinMode(pin_IR, INPUT_PULLUP);
  }
}

void INIT_SERVO()
{
  tca.selectChannel(CHANNEL_PCA9685); delay(10);
  if (!pwm.begin())
  {
    Serial.println("Not Found PCA9685 📌");
    while (1);
  }
  Serial.println("Found PCA9685 ✅");
  pwm.setOscillatorFrequency(27000000); // calibrate nếu cần
  pwm.setPWMFreq(50);  // tần số servo thường là 50Hz
  delay(10);
}

void INIT_GY511()
{
  tca.selectChannel(CHANNEL_GY511); delay(10);
  if (!accel.begin() || !mag.begin()) { // Hàm kiểm tra xem module GY511 có hoạt động hay không?
    Serial.println("Not Found GY511 📌");
    while (1);
  }
  Serial.println("Found GY511 ✅");
  // delay(1000);
}

void INIT_MPU()
{
  tca.selectChannel(CHANNEL_MPU6050); delay(10);
  if (!mpu.begin()) { // Hàm kiểm tra xem module MPU6050 có hoạt động hay không?
    Serial.println("Not Found MPU6050 📌");
    while (1);
  }
  Serial.println("Found MPU6050 ✅");

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  angleX = atan2(accY, accZ) * 180 / PI;
  angleY = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;
  kalmanPitch.setAngle(angleX);
  kalmanRoll.setAngle(angleY);
  timer = micros();
}

void INIT_MCP()
{
  tca.selectChannel(CHANNEL_MCP23017); delay(10);
  if (!mcp.begin_I2C()) // Hàm kiểm tra xem module MCP23017 có hoạt động hay không?
  {
    Serial.println("Not Found MCP23017 📌");
    while(1);
  }
  Serial.println("Found MCP23017 ✅");
}

void INIT_PCA()
{
  if (!tca.begin()) // Hàm kiểm tra xem module TCA548A có hoạt động hay không?
  {
    Serial.println("Not Found TCA9548A 📌");
    while(1); // Dùng để tránh chạy tiếp tục khi có vấn đề kết nối với module
  }
  Serial.println("Found TCA9548A ✅");
}

void INIT_LCD()
{
  tca.selectChannel(CHANNEL_LCD2004); delay(10);
  lcd.init();
  lcd.backlight();
  lcd.setAutoNewLine(false); // Không tự động xuống dòng
}

// ================Khai báo các hàm điều khiển - Control Function================
void CON_SERVO(int F_Ser, int L_Ser, int Step, int angle)
{
  tca.selectChannel(CHANNEL_PCA9685); delay(10);
  uint16_t pulse = angleToPulse(angle);
  for (int i = F_Ser; i < L_Ser; i+=Step) { // Cấp xung PWM cho từng cổng điều khiển trên module PCA9685
    pwm.setPWM(i, 0, pulse);
  }
}

void TEST_L298N_ENCODER_VER1(int speed)
{
  GO_FORWARD(speed);
  TEST_ENCODER_VER1();
}

void TEST_L298N_ENCODER_VER2(int speed)
{
  GO_FORWARD(speed);
  TEST_ENCODER_VER2();
}

void TEST_L298N_ENCODER_VER3(int speed)
{
  GO_FORWARD(speed);
  TEST_ENCODER_VER3();
}

// ================Khai báo các hàm kiểm tra từng module - Test Function================
void TEST_MPU6050()
{
  READ_MPU6050();

  // === LCD ===
  tca.selectChannel(CHANNEL_LCD2004); delay(10);
  lcd.clear();
  lcd.print("TEST MPU6050");
  lcd.setCursor(0, 1);
  lcd.print("Pitch: "); lcd.print(angleX, 1); 
  lcd.setCursor(0, 2);
  lcd.print("Roll: "); lcd.print(angleY, 1);
  delay(500); 
}

void TEST_GY511()
{
  READ_GY511();

  // === Hiển thị lên LCD ===
  tca.selectChannel(CHANNEL_LCD2004); delay(10);
  lcd.setCursor(0, 0);
  lcd.print("TEST GY511");
  lcd.setCursor(0, 1); 
  lcd.print("Pitch: "); lcd.print(angleGYX, 1); 
  lcd.setCursor(0, 2);
  lcd.print("Roll: "); lcd.print(angleGYY, 1);
  lcd.setCursor(0, 3); 
  lcd.print("Yaw: "); lcd.print(angleGYZ, 1); 
  lcd.setCursor(9, 3); 
  lcd.print("| "); lcd.print(angleGYZFiltered, 1);
  delay(500); 
}

void TEST_SERVO(int F_Ser, int L_Ser, int Step, int angle)  
{
  tca.selectChannel(CHANNEL_LCD2004); delay(10);
  lcd.setCursor(0, 0);
  lcd.print("TEST SERVO");
  lcd.setCursor(0, 1);
  lcd.print("SERVO at pins:"); lcd.print(F_Ser); lcd.print(" -> "); lcd.print(L_Ser);
  lcd.setCursor(0, 2);
  lcd.print("Rotate: "); lcd.print(angle); lcd.print(" degree");

  tca.selectChannel(CHANNEL_PCA9685); delay(10);
  for (int angle = 0; angle <= 180; angle += 5) {
    CON_SERVO(F_Ser, L_Ser, Step, angle);
    delay(30);
  }
  for (int angle = 180; angle >= 0; angle -= 5) {
    CON_SERVO(F_Ser, L_Ser, Step, angle);
    delay(30);
  }
}

void TEST_ENCODER_VER1() // Encoder + MCP
{  
  READ_ENCODER(1);

  tca.selectChannel(CHANNEL_LCD2004); delay(10);
  // In hàng encoder bên phải là dòng 2 --> Đếm xung
  lcd.setCursor(0, 0);
  lcd.print("TEST ENCODER VER1");
  lcd.setCursor(0, 1);
  lcd.print("E0"); lcd.print(":"); lcd.print(encoderCounts[0]);
  lcd.setCursor(9, 1);
  lcd.print("E1"); lcd.print(":"); lcd.print(encoderCounts[1]);

  // In hàng encoder bên phải là dòng 3 --> Đếm xung
  lcd.setCursor(0, 2);
  lcd.print("E2"); lcd.print(":"); lcd.print(encoderCounts[2]);
  lcd.setCursor(9, 2);
  lcd.print("E3"); lcd.print(":"); lcd.print(encoderCounts[3]);

  // In hàng encoder 2 bên là dòng 4 --> Đếm số vòng quay
  lcd.setCursor(0, 3);
  lcd.print(rotate_per_eva[0]);
  lcd.setCursor(5, 3);
  lcd.print(rotate_per_eva[1]);
  lcd.setCursor(10, 3);
  lcd.print(rotate_per_eva[2]);
  lcd.setCursor(15, 3);
  lcd.print(rotate_per_eva[3]);
}

void TEST_ENCODER_VER2() // Encoder + GPIO không có ngắt ngoài
{
  READ_ENCODER(2);
  
  tca.selectChannel(CHANNEL_LCD2004); delay(10);
  lcd.setCursor(0, 0);
  lcd.print("TEST ENCODER VER2");

  // In hàng encoder bên phải là dòng 2 --> Đếm xung
  lcd.setCursor(0, 1);
  lcd.print("E0"); lcd.print(":"); lcd.print(encoderCounts[0]);
  lcd.setCursor(9, 1);
  lcd.print("E1"); lcd.print(":"); lcd.print(encoderCounts[1]);

  // In hàng encoder bên phải là dòng 3 --> Đếm xung
  lcd.setCursor(0, 2);
  lcd.print("E2"); lcd.print(":"); lcd.print(encoderCounts[2]);
  lcd.setCursor(9, 2);
  lcd.print("E3"); lcd.print(":"); lcd.print(encoderCounts[3]);

  // In hàng encoder 2 bên là dòng 4 --> Đếm số vòng quay
  lcd.setCursor(0, 3);
  lcd.print(rotate_per_eva[0]);
  lcd.setCursor(5, 3);
  lcd.print(rotate_per_eva[1]);
  lcd.setCursor(10, 3);
  lcd.print(rotate_per_eva[2]);
  lcd.setCursor(15, 3);
  lcd.print(rotate_per_eva[3]);
}

void TEST_ENCODER_VER3() // Encoder + GPIO không có ngắt ngoài
{
  READ_ENCODER(3);

  tca.selectChannel(CHANNEL_LCD2004); delay(10);
  lcd.setCursor(0, 0);
  lcd.print("TEST ENCODER VER3");

  // In hàng encoder bên phải là dòng 2 --> Đếm xung
  lcd.setCursor(0, 1);
  lcd.print("E0"); lcd.print(":"); lcd.print(encoderCounts[0]);
  lcd.setCursor(9, 1);
  lcd.print("E1"); lcd.print(":"); lcd.print(encoderCounts[1]);

  // In hàng encoder bên phải là dòng 3 --> Đếm xung
  lcd.setCursor(0, 2);
  lcd.print("E2"); lcd.print(":"); lcd.print(encoderCounts[2]);
  lcd.setCursor(9, 2);
  lcd.print("E3"); lcd.print(":"); lcd.print(encoderCounts[3]);

  // In hàng encoder 2 bên là dòng 4 --> Đếm số vòng quay
  lcd.setCursor(0, 3);
  lcd.print(rotate_per_eva[0]);
  lcd.setCursor(5, 3);
  lcd.print(rotate_per_eva[1]);
  lcd.setCursor(10, 3);
  lcd.print(rotate_per_eva[2]);
  lcd.setCursor(15, 3);
  lcd.print(rotate_per_eva[3]);

  delay(500);
}

void TEST_LCD2004() // Chạy tốt
{
  tca.selectChannel(CHANNEL_LCD2004); delay(10);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TEST LCD");
  delay(1000);
  for (uint8_t row = 0; row <= 3; row++)
  {
    for (uint8_t col = 0; col <= 20; col++)
    {
      lcd.clear();
      lcd.setCursor(col, row);
      lcd.print("X");
      delay(1000);
    }  
  }
}

void TEST_LINE() // Chạy tốt
{
  READ_LINE();

  tca.selectChannel(CHANNEL_LCD2004);
  lcd.setCursor(0, 0);
  lcd.print("TEST LINE");
  lcd.setCursor(0, 1);
  lcd.print("IR: ");
  for (int i = 0; i < 5; i++) 
  {
    lcd.print(lineState[i]); // In trạng thái 0 hoặc 1
    lcd.print(" ");
  }
  delay(100);
}

void TEST_L298N_VER1()
{
  tca.selectChannel(CHANNEL_LCD2004); delay(10);
  lcd.clear();
  lcd.setCursor(0, 0); 
  lcd.print("TEST L298N VER1");

  tca.selectChannel(CHANNEL_MCP23017); delay(10);
  mcp.digitalWrite(8, HIGH); mcp.digitalWrite(9, LOW);
  mcp.digitalWrite(10, HIGH); mcp.digitalWrite(11, LOW);
  delay(1000);
  mcp.digitalWrite(8, LOW); mcp.digitalWrite(9, HIGH);
  mcp.digitalWrite(10, LOW); mcp.digitalWrite(11, HIGH);
  delay(1000);
}

void TEST_L298N_VER2(int speed)
{
  tca.selectChannel(CHANNEL_LCD2004); delay(10);
  lcd.setCursor(0, 0); 
  lcd.print("TEST L298N VER2");
  lcd.setCursor(0, 1); 
  lcd.print("SPEED:"); lcd.print(speed);

  GO_FORWARD(speed);
  delay(5000);
  GO_BACKWARD(speed);
  delay(5000);
  SPIN_LEFT(speed);
  delay(2000);
  SPIN_RIGHT(speed);
  delay(2000);
}

void TEST_DS1302() // Chạy tốt
{
  READ_DS1302();
}

void TEST_MPU6050_GY511_KALMAN() // Chạy tốt
{
  READ_MPU6050();
  READ_GY511();

  // === LCD ===
  tca.selectChannel(CHANNEL_LCD2004); delay(10);
  lcd.clear();
  lcd.print("TEST MPU_GY_KALMAN");
  lcd.setCursor(0, 1);
  lcd.print("MPU X:"); lcd.print(angleX, 1); lcd.print(" Y:"); lcd.print(angleY, 1);

  lcd.setCursor(0, 2); 
  lcd.print("GY  X:"); lcd.print(angleGYX, 1); lcd.print(" Y:"); lcd.print(angleGYY, 1);

  // In RAM còn lại
  // Serial.printf("Heap RAM: %u bytes\n", ESP.getFreeHeap());
  delay(500);  
}

bool TEST_DESIRED_DISTANCE (float des_distance)
{
  float Distance_Error = des_distance - 15.0;                          // Có thể sử dụng PID để chính xác
  float RotateDes = (float)Distance_Error / Pwheel;                    // Số vòng cần đi với quãng đường có sẵn
  int reached = 0;
  float totalRotations = 0;

  for (uint8_t i = 0; i < NUM_ENCODERS; i++)                           // Ghi lại số vòng quay của 4 ENCODERs
  {
    rotate_per_eva[i] = (float)encoderCounts[i] / pulse_def;
    totalRotations += rotate_per_eva[i];

    if (rotate_per_eva[i] >= RotateDes) reached++;
  }

  float avgRotations = totalRotations / NUM_ENCODERS;             // Trung bình số vòng của 4 encoder
  float distance_traveled = avgRotations * Pwheel;                // Khoảng cách đã đi được

  tca.selectChannel(CHANNEL_LCD2004); delay(10);
  lcd.setCursor(0, 0);
  lcd.print("DESIRED DISTANCE");
  lcd.setCursor(0, 1);
  lcd.printf("Distance: %dcm", (int)des_distance);

  if (reached >= 4) // đủ 4 bánh tới khoảng cách mong muốn 
  { 
    STOP_ALL();
    lcd.setCursor(0, 2);
    lcd.printf("Completed - %.1f cm", des_distance);
    delay(1000);
    return true;
  } 
  else 
  {
    GO_FORWARD(speed);
    lcd.setCursor(0, 2);
    lcd.printf("Travelled - %.2f cm", distance_traveled);
  }
  return false;
}

bool TEST_DESIRED_ROTATION(float DesWheelRotate)
{
  int reached = 0;
  int minSpeed = 180;       // Tốc độ nhỏ nhất để động cơ vẫn quay
  int maxSpeed = 255;      // Tốc độ lớn nhất
  float totalRotations = 0;
  const float Kp = 250.0;

  for (uint8_t i = 0; i < NUM_ENCODERS; i++) 
  {
    rotate_per_eva[i] = (float)encoderCounts[i] / pulse_def;
    totalRotations += rotate_per_eva[i];

    if (rotate_per_eva[i] >= DesWheelRotate) reached++;
  }
  float avgRotation = totalRotations/4.0;
  float rot_error = DesWheelRotate - avgRotation;

  if (reached >= 4) 
  {
    STOP_ALL();
    // Reset tất cả thông số di chuyển
    totalRotations = 0;
    reached = 0;
    for(int i=0; i<4; i++)
    {
      rotate_per_eva[i] = 0;   
      encoderCounts[i] = 0;   
    }
    return true;
    delay(1000);
  }
  else 
  {
    GO_FORWARD(speed);  
    speed = constrain(abs(rot_error) * Kp, minSpeed, maxSpeed);
  }
  return false;
}

bool TEST_DESIRED_ANGLE(float des_angle, int max_angle)
{
  int minSpeed = 195;       // Tốc độ nhỏ nhất để động cơ vẫn quay
  int maxSpeed = 255;      // Tốc độ lớn nhất
  float Kp = 4.7;          // Hệ số tỉ lệ, có thể điều chỉnh

  float yawError, currentYaw;
  READ_GY511();

  // tca.selectChannel(CHANNEL_LCD2004); delay(10);
  // lcd.setCursor(0, 0);
  // lcd.printf("DESIRED ANGLE - %d*", (int)des_angle);

  // Chỉ tính góc mục tiêu 1 lần
  if (!YawInitSave)
  {
    startYaw = angleGYZ;
    targetYaw = (max_angle == 0) ? fmod(des_angle - startYaw + 360.0, 360.0) : des_angle - startYaw;
    YawInitSave = true;
    delay(1000);
  }

  if (!stop_flag)            // Khi đã xoay tới vị trí thì dừng
  {
    // Tính sai số
    currentYaw = (max_angle == 0) ? fmod(angleGYZ + 360.0, 360.0) : angleGYZ;
    currentYaw += 3;                                                            // 6 độ trượt
    yawError = targetYaw - currentYaw;

    speed = constrain(abs(yawError) * Kp, minSpeed, maxSpeed);

    // lcd.setCursor(0, 1);
    // lcd.printf("CY: %.1f | YE %.1f", currentYaw, yawError);
    // lcd.setCursor(0, 2);
    // lcd.printf("TY: %.1f | YS %.1f", targetYaw, startYaw);

    if (abs(yawError) < 4.0) 
    {
      STOP_ALL();
      stop_flag = true; 
      delay(1000);
      return true; 
    } 
    else 
    {
      if (yawError > 0) SPIN_RIGHT(speed);
      else SPIN_LEFT(speed);
    }
    return false;
  }
}


// ================Khai báo các hàm điều khiển động cơ - Motor Control Function================
void SPIN_LEFT(int speed)
{
  // Serial.println("SPIN LEFT");
  RIGHT_MOTOR_FORWARD(speed);
  LEFT_MOTOR_BACKWARD(speed);
}

void SPIN_RIGHT(int speed)
{
  // Serial.println("SPIN RIGHT");
  RIGHT_MOTOR_BACKWARD(speed);
  LEFT_MOTOR_FORWARD(speed);
}

void GO_FORWARD(int speed)
{
  // Serial.println("GO FORWARD");
  LEFT_MOTOR_FORWARD(speed);
  RIGHT_MOTOR_FORWARD(speed);
}

void GO_BACKWARD(int speed)
{
  // Serial.println("GO BACKWARD");
  LEFT_MOTOR_BACKWARD(speed);
  RIGHT_MOTOR_BACKWARD(speed);
}

void STOP_ALL()
{
  // Serial.println("STOP ALL");
  digitalWrite(LEFT_A, LOW);
  digitalWrite(LEFT_B, LOW);
  analogWrite(EN_LEFT, 0);

  digitalWrite(RIGHT_A, LOW);
  digitalWrite(RIGHT_B, LOW);
  analogWrite(EN_RIGHT, 0);
}

void LEFT_MOTOR_FORWARD(int speed)
{
  // Serial.println("LEFT MOTOR FORWARD");
  speed = constrain(speed, MIN_SPEED, MAX_SPEED); //đảm báo giá trị nằm trong một khoảng từ 0 - MAX_SPEED

  digitalWrite(LEFT_B, 1); 
  digitalWrite(LEFT_A, 0); 
  analogWrite(EN_LEFT, speed);
}

void LEFT_MOTOR_BACKWARD(int speed)
{
  // Serial.println("LEFT MOTOR BACKWARD");
  speed = constrain(speed, MIN_SPEED, MAX_SPEED); //đảm báo giá trị nằm trong một khoảng từ 0 - MAX_SPEED

  digitalWrite(LEFT_A, 1); 
  digitalWrite(LEFT_B, 0); 
  analogWrite(EN_LEFT, speed);
}

void RIGHT_MOTOR_FORWARD(int speed)
{
  // Serial.println("RIGHT MOTOR FORWARD");
  speed = constrain(speed, MIN_SPEED, MAX_SPEED); //đảm báo giá trị nằm trong một khoảng từ 0 - MAX_SPEED

  digitalWrite(RIGHT_B, 1); 
  digitalWrite(RIGHT_A, 0); 
  analogWrite(EN_RIGHT, speed);
}

void RIGHT_MOTOR_BACKWARD(int speed)
{
  // Serial.println("RIGHT MOTOR BACKWARD");
  speed = constrain(speed, MIN_SPEED, MAX_SPEED); //đảm báo giá trị nằm trong một khoảng từ 0 - MAX_SPEED

  digitalWrite(RIGHT_A, 1); 
  digitalWrite(RIGHT_B, 0); 
  analogWrite(EN_RIGHT, speed);
}
