#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
//Define for Ultrasonic sensor
#define TRIG_PIN 23
#define ECHO_PIN 22
#define TIME_OUT 5000
#define Threshold_distance 40
long duration, distanceCm;

//Define PIN to control Motor control
#define ENA 13                              // Enable A for Motor A
#define IN1 12                              // Input 1 of Motor A
#define IN2 14                              // Input 2 of Motor A
#define ENB 25                              // Enable A for Motor B
#define IN3 27                              // Input 1 of Motor B
#define IN4 26                              // Input 2 of Motor B
//Define PIN to control Motor control
#define ENC 21                              // Enable A for Motor C
#define IN5 19                              // Input 1 of Motor C
#define IN6 18                              // Input 2 of Motor C
#define END 15                              // Enable A for Motor D
#define IN7 5                              // Input 1 of Motor D
#define IN8 2                              // Input 2 of Motor D
//Definition Pins of encoder 
#define EncoderA 35                         // Encoder channel A of Motor A
#define EncoderB 34                          // Encoder channel B of Motor A
#define EncoderC 32                         // Encoder channel A of Motor B                
#define EncoderD 33                         // Encoder channel B of Motor B                

//Config Servo PIN GPIO4
#define PIN_SG90 4                          // Output pin used GPIO 4
Servo sg90;

// Information to config wifi
const char *ssid = "Nha 198";
const char *password = "99999999";
// CONFIG STATIC IP
IPAddress local_IP(192, 168, 1, 184); 
IPAddress gateway(192, 168, 1, 1);     
IPAddress subnet(255, 255, 255, 0);    
IPAddress primaryDNS(8, 8, 8, 8);      
IPAddress secondaryDNS(8, 8, 4, 4); 

// Create the plant AsyncWebServer to run on gate 80
AsyncWebServer server(80);
//Config UART PIN 
SoftwareSerial mySerial(16, 17);            // GPIO16-RX GPIO17-TX
static float prev_e_A = 0;  // Giá trị trước đó của e_A
static float prev_e_B = 0;  // Giá trị trước đó của e_B
static int stable_count = 0;  // Đếm số lần không đổi
//Definition parameter for PID function
long prevT_A = 0;                             // Previous time for Rotate A
long prevT_B = 0;                             // Previous time for Rotate B

float eprev_A = 0;                            // Error Previous for Rotate A
float eprev_B = 0;                            // Error Previous for Rotate B

float enitegral_A = 0;                        // Total Error for Rotate A
float enitegral_B = 0;                        // Total Error for Rotate B

long prevT_str_A = 0;                         // Previous time for Straight A
long prevT_str_B = 0;                         // Previous time for Straight B

float eprev_str_A = 0;                        // Error Previous for Straight A
float eprev_str_B = 0;                        // Error Previous for Straight B

float enitegral_str_A = 0;                    // Total Error for Straight A
float enitegral_str_B = 0;                    // Total Error for Straight B
float Kp_RT = 4;
float Ki_RT = 0.8;
float Kd_RT = 0;
float Kp_STR = 4;
float Ki_STR = 0.8;
float Kd_STR = 0;
//Counter Encoder
int Position_MotorA = 0;                    // Position of Motor A read by Encoder A
int Position_MotorB = 0;                    // Position of Motor B read by Encoder B

int state_pick = 0;
int state_rotate = 0;                       // State to get the angle
int state_auto = 0;                         // State to auto moving
int state_str = 0;                          // State to go straight
int err_angle = 10;
String state_fb ="";                        // State to get Feedback
String state_RT ="";                        // State to Rotate

int distance = 0;                           // Variable of distance
int P_rotate_A = 0;
int P_rotate_B = 0;
int angle = 0;                              // Variable of angle
int count = 0;                              // Variable of count to check position Rotate
int count_pwm = 0;
int count_str = 0;                          // Variable of count to check position Straight
int count_balls = 0;
int lastState = 1;                          // Keep the previous state of GPIO 35
String check_FB = "";

int zero = 0;
int one = 1;
int three = 3;
// Variable to save data from webserver by post method
String receivedData = "";                   // Data control moving for Tenibot
String PwmDataAction ="";                   // Data of speed percents control action for Tenibot
String PwmDataPickBalls ="";                // Data of speed percents control pick-up balls
String statePick = "";                      // Data of status for pick-up balls
String stateAuto = "";                      // Flag for state Auto
String flag_uart = "False";                 // Flag for UART
int flag_uart_check = 0;
int flag_uart_check_state = 0;
int flag_Servo = 0;                         // Flag for Servo
// Function to handle get data
void handleGetData(AsyncWebServerRequest *request)
{
    request->send(200, "text/plain", receivedData);
}
// Function to handler request GET at "/getValue"
void handleGetValue(AsyncWebServerRequest *request)
{
    // Creat Json string included digitalValue
    String jsonResponse = "{\"digitalValue\": " + String(count_balls) + "}";

    // Send data Jason Form to client
    request->send(200, "application/json", jsonResponse);
}
// Function to handle request by Method Post to /submit
void handleSubmit(AsyncWebServerRequest *request)
{
    if (request->hasParam("data", true))
    {
        String data = request->getParam("data", true)->value();
        receivedData = data;
        Serial.println("Dữ liệu Move: " + receivedData);
        request->redirect("/");
    }
    else
    {
        request->send(400, "text/plain", "Bad Request");
    }
}

// Function to handle request by Method Post to /PwmAction
void handlePwmAction(AsyncWebServerRequest *request){
    if (request->hasParam("data", true))
    { 
        String data = request->getParam("data", true)->value();
        PwmDataAction = data;
        Serial.println("Dữ liệu PWM: " + PwmDataAction);
        request->redirect("/");
    }
    else
    {
        request->send(400, "text/plain", "Bad Request");
    }
}

// Function to handle request by Method Post to /PickPwm
void handlePwmPickBalls(AsyncWebServerRequest *request){
    if (request->hasParam("data", true))
    { 
        String data = request->getParam("data", true)->value();
        PwmDataPickBalls = data;
        Serial.println("Dữ liệu PWM Pick: " + PwmDataPickBalls);
        request->redirect("/");
    }
    else
    {
        request->send(400, "text/plain", "Bad Request");
    }
}
void handlerEncoder_Left(){
    if (digitalRead(EncoderA) == digitalRead(EncoderB)) {
        Position_MotorA++;
    } else {
        Position_MotorA--; 
    }
}
void handlerEncoder_Right(){
    if (digitalRead(EncoderC) == digitalRead(EncoderD)) {
        Position_MotorB++; 
    } else {
        Position_MotorB--; 
    }
}
void Task_handler_count_balls(void *pvParameters){
    pinMode(39,INPUT);
    while (1) {
        // Read present value from GPIO 35
        int digitalValue = digitalRead(39);

        if (lastState == 1 && digitalValue == 0) {
            count_balls++;
        }

        // Cập nhật trạng thái trước đó
        lastState = digitalValue;

        // Delay để giảm tải CPU
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
// Function to handle request by Method Post to /PickState
void handlePickBalls(AsyncWebServerRequest *request){
    if (request->hasParam("data", true))
    { // true để tìm trong POST
        String data = request->getParam("data", true)->value();
        statePick = data;
        Serial.println("Dữ liệu State pick: " + statePick);
        request->redirect("/");
    }
    else
    {
        request->send(400, "text/plain", "Bad Request");
    }
}
// Function to handle request by Method Post to /PickState
void handleAutoMode(AsyncWebServerRequest *request){
    if (request->hasParam("data", true))
    { // true để tìm trong POST
        String data = request->getParam("data", true)->value();
        stateAuto = data;
        Serial.println("Dữ liệu State Auto: " + stateAuto);
        request->redirect("/");
    }
    else
    {
        request->send(400, "text/plain", "Bad Request");
    }
}
void Task_Handler_Uart_send(void *pvParameters) {
    while(1){
        if(stateAuto == "True" && flag_uart == "False"){
            mySerial.println(one);
            flag_uart = "True";
            vTaskDelay(100 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
        }else if(stateAuto == "False" && flag_uart == "True"){
            mySerial.println(zero);
            flag_uart = "False";
            vTaskDelay(100 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
        }
    }
}
void Task_Handler_Uart_recieve(void *pvParameters) {
    while (1) {
        if (mySerial.available() && stateAuto == "True" ) {
            String dataString = mySerial.readString();
            int separatorIndex = dataString.indexOf(';');
            int separatorIndex_FB = dataString.indexOf('@');
            if (separatorIndex != -1) {  // Kiểm tra nếu ký tự ';' tồn tại
                state_auto = dataString.substring(0, separatorIndex).toInt();
                state_rotate = state_auto;
                distance = dataString.substring(separatorIndex + 1).toInt();
                flag_uart_check_state = 1;
                vTaskDelay(100 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
                Position_MotorA = 0;
                Position_MotorB = 0;
                P_rotate_A = 0;
                P_rotate_B = 0;
                vTaskDelay(10 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
            }else if(separatorIndex_FB != -1){
                check_FB = dataString.substring(0,separatorIndex_FB);
                state_fb = check_FB;
                flag_uart_check = 1;
                vTaskDelay(100 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
                Position_MotorA = 0;
                Position_MotorB = 0;
                P_rotate_A = 0;
                P_rotate_B = 0;
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
    }
}
void Task_handler_send_uart_state(void *pvParameters){
    while(1){
        if(flag_uart_check == 1){
            mySerial.println(one);
            flag_uart_check = 0;
            vTaskDelay(100 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
        }
        if(flag_uart_check_state == 1){
            mySerial.println(zero);
            flag_uart_check_state = 0;
            vTaskDelay(100 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
    }
}
void Task_handler_Servo(void *pvParameters){
    // Đính kèm servo với chân PWM
    sg90.setPeriodHertz(50);          // PWM frequency for SG90
    sg90.attach(PIN_SG90, 500, 2500); // Minimum and maximum pulse width (in µs) to go from 0° to 180
    while (1)
    {
        for(int i = 0; i < 180; i++){
            if(flag_Servo == 0){
                sg90.write(i);
                if(state_rotate == 1){
                    noInterrupts(); // Tạm thời tắt ngắt để đồng bộ hóa dữ liệu
                    angle = i - err_angle;
                    Position_MotorA = 0;
                    Position_MotorB = 0;
                    P_rotate_A = 0;
                    P_rotate_B = 0;
                    state_rotate = 0; // Đặt lại cờ
                    interrupts();   // Bật lại ngắt
                    vTaskDelay(200 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
                    flag_Servo = 1;
                }
            }
            vTaskDelay(100 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
        }
        for(int i = 180; i > 0; i--){
            if(flag_Servo == 0){
                sg90.write(i);
                if(state_rotate == 1){
                    noInterrupts(); // Tạm thời tắt ngắt để đồng bộ hóa dữ liệu
                    angle = i;
                    Position_MotorA = 0;
                    Position_MotorB = 0;
                    P_rotate_A = 0;
                    P_rotate_B = 0;
                    state_rotate = 0; // Đặt lại cờ
                    interrupts();   // Bật lại ngắt
                    vTaskDelay(200 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
                    flag_Servo = 1;
                }
            }
            vTaskDelay(100 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
        }
    }
}
void Task_handler_check_Servo(void *pvParameters){
    while(1){
            for(int i = 90; i >= 0; i--){
                if(state_fb == "R"){
                    sg90.write(i);
                    if(state_rotate == 1){
                        noInterrupts(); // Tạm thời tắt ngắt để đồng bộ hóa dữ liệu
                        angle = i;
                        Position_MotorA = 0;
                        Position_MotorB = 0;
                        P_rotate_A = 0;
                        P_rotate_B = 0;
                        state_rotate = 0; // Đặt lại cờ
                        interrupts();   // Bật lại ngắt
                        vTaskDelay(200 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
                        state_fb = "";
                    }
                    vTaskDelay(200 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
                }
            }
            for(int i = 0; i <= 90; i++){
                if(state_fb == "R"){
                    sg90.write(i);
                    if(state_rotate == 1){
                        noInterrupts(); // Tạm thời tắt ngắt để đồng bộ hóa dữ liệu
                        angle = i;
                        Position_MotorA = 0;
                        Position_MotorB = 0;
                        P_rotate_A = 0;
                        P_rotate_B = 0;
                        state_rotate = 0; // Đặt lại cờ
                        interrupts();   // Bật lại ngắt
                        vTaskDelay(200 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
                        state_fb = "";
                    }
                    vTaskDelay(200 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
                }
            }
            for(int i = 90; i <= 180; i++){
                if(state_fb == "L"){
                    sg90.write(i);
                    if(state_rotate == 1){
                        noInterrupts(); // Tạm thời tắt ngắt để đồng bộ hóa dữ liệu
                        angle = i;
                        Position_MotorA = 0;
                        Position_MotorB = 0;
                        P_rotate_A = 0;
                        P_rotate_B = 0;
                        state_rotate = 0; // Đặt lại cờ
                        interrupts();   // Bật lại ngắt
                        vTaskDelay(200 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
                        state_fb = "";
                    }
                    vTaskDelay(200 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
                }  
            }
            for(int i = 180; i >= 90; i--){
                if(state_fb == "L"){
                    sg90.write(i);
                    if(state_rotate == 1){
                        noInterrupts(); // Tạm thời tắt ngắt để đồng bộ hóa dữ liệu
                        angle = i;
                        Position_MotorA = 0;
                        Position_MotorB = 0;
                        P_rotate_A = 0;
                        P_rotate_B = 0;
                        state_rotate = 0; // Đặt lại cờ
                        state_fb = "";
                        interrupts();   // Bật lại ngắt
                        vTaskDelay(200 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
                    }
                    vTaskDelay(200 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
                } 
            }
            if(state_fb == "T"){
                    state_pick = 1;
                    state_str = 1;
                    Position_MotorA = 0;
                    Position_MotorB = 0;
                    state_fb = "";
                    vTaskDelay(10 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
            }
            vTaskDelay(200 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
        }
}

// Function to control Motor A by PID
void control_motor_A_RTR(int Pwm){
    digitalWrite(ENA,HIGH);                     
    if (Pwm > 0) {
        analogWrite(IN2, Pwm);
        analogWrite(IN1, 0);
    }
    else if (Pwm == 0) {
        analogWrite(IN1, 0);
        analogWrite(IN2, 0);
    }
    else {
        analogWrite(IN2, 0);
        analogWrite(IN1, -Pwm);
    }
}
// Function to control Motor B by PID sate Rotate
void control_motor_B_RTR(int Pwm){
    digitalWrite(ENB,HIGH);
    if (Pwm > 0) {
        analogWrite(IN4, Pwm);
        analogWrite(IN3, 0);
    }
    else if (Pwm == 0) {
        analogWrite(IN3, 0);
        analogWrite(IN4, 0);
    }
    else {
        analogWrite(IN4, 0);
        analogWrite(IN3, -Pwm);
    }
}
// Function to control Motor B by PID state straight
void control_motor_B_str(int Pwm){
    digitalWrite(ENB,HIGH);
    digitalWrite(ENA,HIGH);                     //
    if (Pwm > 0) {
        analogWrite(IN4, Pwm);
        analogWrite(IN3, 0);
        analogWrite(IN2, Pwm);
        analogWrite(IN1, 0);
    }
    else if (Pwm == 0) {
        analogWrite(IN3, 0);
        analogWrite(IN4, 0);
        analogWrite(IN1, 0);
        analogWrite(IN2, 0);
    }
    else {
        analogWrite(IN4, 0);
        analogWrite(IN3, -Pwm);
        analogWrite(IN2, 0);
        analogWrite(IN1, -Pwm);
    }
}
void control_motor_A_str(int Pwm){
    digitalWrite(ENB,HIGH);
    digitalWrite(ENA,HIGH);                     //
    if (Pwm > 0) {
        analogWrite(IN4, Pwm);
        analogWrite(IN3, 0);
        analogWrite(IN2, Pwm);
        analogWrite(IN1, 0);
    }
    else if (Pwm == 0) {
        analogWrite(IN3, 0);
        analogWrite(IN4, 0);
        analogWrite(IN1, 0);
        analogWrite(IN2, 0);
    }
    else {
        analogWrite(IN4, 0);
        analogWrite(IN3, -Pwm);
        analogWrite(IN2, 0);
        analogWrite(IN1, -Pwm);
    }
}
void Task_handler_PID(void *pvParameters){
    while(1){
        if(state_auto == 1 && stateAuto == "True"){ // || state_RT == "L" || state_RT == "R"
            Serial.println(angle);
                if(angle < 90){
                    P_rotate_A = (((59400 * (90 - angle)) / (65 * 180)));
                    P_rotate_B = -(((59400 * (90 - angle)) / (65 * 180)));
                    long currT_A = micros();
                    long currT_B = micros();
                    float deltaT_A = ((float)(currT_A - prevT_A)) / 1000000.0;
                    float deltaT_B = ((float)(currT_B - prevT_B)) / 1000000.0;
                    prevT_A = currT_A;
                    prevT_B = currT_B;
                    // Lấy giá trị hiện tại của Position_MotorA
                    float e_A = Position_MotorA - P_rotate_A;
                    float e_B = Position_MotorB - P_rotate_B;
                    float dedt_A = (e_A - eprev_A) / deltaT_A;
                    float dedt_B = (e_B - eprev_B) / deltaT_B;
                    enitegral_A += e_A * deltaT_A;
                    enitegral_B += e_B * deltaT_B;
                    // Tính toán tín hiệu điều khiển PID
                    float u_A = Kp_RT * e_A + Ki_RT * dedt_A + Kd_RT * enitegral_A;
                    float u_B = Kp_RT * e_B + Ki_RT * dedt_B + Kd_RT * enitegral_B;
                    u_A = (int)u_A;
                    u_B = (int)u_B;
                    // if(abs(u_A) < 130 && u_A > 0){
                    //     u_A = 130;
                    // }else if(abs(u_A) < 130 && u_A < 0){
                    //     u_A = -130;
                    // }
                    // if(abs(u_B) < 130 && u_B > 0){
                    //     u_B = 130;
                    // }else if(abs(u_B) < 130 && u_B < 0){
                    //     u_B = -130;
                    // }
                    if(u_A >= 255){
                        u_A = 255;
                    }else if(u_A <= -255){
                        u_A = -255;
                    }
                    if(u_B >= 255){
                        u_B = 255;
                    }else if(u_B <= -255){
                        u_B = -255;
                    }
                    control_motor_A_RTR(u_A);
                    control_motor_B_RTR(u_B);
                    eprev_A = e_A;
                    eprev_B = e_B;
                    // if(fabs(e_A) <= 10 && fabs(e_B) <= 10){/
                    //     count++;
                    //     vTaskDelay(50 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
                    //     if(count >= 10){
                    //         count = 0;
                    //         Position_MotorA = 0;
                    //         Position_MotorB = 0;
                    //         control_motor_A_RTR(0);
                    //         control_motor_B_RTR(0);
                    //         P_rotate_A = 0;
                    //         P_rotate_B = 0;
                    //         state_rotate = 0;
                    //         state_fb = "";
                    //         state_RT = "";
                    //         sg90.write(90);
                    //         mySerial.println("3");
                    //         vTaskDelay(100 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU          
                    //         state_auto = 0;
                    //     }
                    // }
                    if(fabs(e_A - prev_e_A) <= 1 && fabs(e_B - prev_e_B) <= 1) { // Kiểm tra sự thay đổi nhỏ
                        stable_count++;
                        vTaskDelay(50 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
                        if(stable_count >= 10) {  // Nếu không đổi trong 10 lần kiểm tra liên tiếp
                            stable_count = 0;
                            Position_MotorA = 0;
                            Position_MotorB = 0;
                            control_motor_A_RTR(0);
                            control_motor_B_RTR(0);
                            P_rotate_A = 0;
                            P_rotate_B = 0;
                            state_rotate = 0;
                            state_fb = "";
                            sg90.write(90);
                            mySerial.println(three);
                            vTaskDelay(100 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU          
                            state_auto = 0;
                        }
                    }
                    prev_e_A = e_A;
                    prev_e_B = e_B;
                    Serial.print("Current Position RTL: ");
                    Serial.print(Position_MotorA);
                    Serial.print(" Target Position RTL: ");
                    Serial.print(P_rotate_A);
                    Serial.print(" PWM: ");
                    Serial.println(u_A);
                    Serial.print("Current Position RTR: ");
                    Serial.print(Position_MotorB);
                    Serial.print(" Target Position RTR: ");
                    Serial.print(P_rotate_B);
                    Serial.print(" PWM: ");
                    Serial.println(u_B);
                }else if(angle > 90){
                    P_rotate_A = -(((59400 * (angle - 90)) / (65 * 180)));
                    P_rotate_B = (((59400 * (angle - 90)) / (65 * 180)));
                    long currT_A = micros();
                    long currT_B = micros();
                    float deltaT_A = ((float)(currT_A - prevT_A)) / 1000000.0;
                    float deltaT_B = ((float)(currT_B - prevT_B)) / 1000000.0;
                    prevT_A = currT_A;
                    prevT_B = currT_B;
                    // Lấy giá trị hiện tại của Position_MotorA
                    float e_A = Position_MotorA - P_rotate_A;
                    float e_B = Position_MotorB - P_rotate_B;
                    float dedt_A = (e_A - eprev_A) / deltaT_A;
                    float dedt_B = (e_B - eprev_B) / deltaT_B;
                    enitegral_A += e_A * deltaT_A;
                    enitegral_B += e_B * deltaT_B;
                    // Tính toán tín hiệu điều khiển PID
                    float u_A = Kp_RT * e_A + Ki_RT * dedt_A + Kd_RT * enitegral_A;
                    float u_B = Kp_RT * e_B + Ki_RT * dedt_B + Kd_RT * enitegral_B;
                    u_A = (int)u_A;
                    u_B = (int)u_B;
                    // if(abs(u_A) < 130 && u_A > 0){
                    //     u_A = 130;
                    // }else if(abs(u_A) < 130 && u_A < 0){
                    //     u_A = -130;
                    // }
                    // if(abs(u_B) < 130 && u_B > 0){
                    //     u_B = 130;
                    // }else if(abs(u_B) < 130 && u_B < 0){
                    //     u_B = -130;
                    // }
                    if(u_A >= 255){
                        u_A = 255;
                    }else if(u_A <= -255){
                        u_A = -255;
                    }
                    if(u_B >= 255){
                        u_B = 255;
                    }else if(u_B <= -255){
                        u_B = -255;
                    }
                    control_motor_A_RTR(u_A);
                    control_motor_B_RTR(u_B);
                    eprev_A = e_A;
                    eprev_B = e_B;
                    // if(fabs(e_A) <= 10 && fabs(e_B) <= 10){
                    //     count++;
                    //     vTaskDelay(50 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
                    //     if(count >= 10){
                    //         count = 0;
                    //         Position_MotorA = 0;
                    //         Position_MotorB = 0;
                    //         control_motor_A_RTR(0);
                    //         control_motor_B_RTR(0);
                    //         P_rotate_A = 0;
                    //         P_rotate_B = 0;
                    //         state_rotate = 0;
                    //         state_fb = "";
                    //         sg90.write(90);
                    //         mySerial.println("3");
                    //         vTaskDelay(100 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU          
                    //         state_auto = 0;
                    //     }
                    // }
                    if(fabs(e_A - prev_e_A) <= 1 && fabs(e_B - prev_e_B) <= 1) { // Kiểm tra sự thay đổi nhỏ
                        stable_count++;
                        vTaskDelay(50 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
                        if(stable_count >= 10) {  // Nếu không đổi trong 10 lần kiểm tra liên tiếp
                            stable_count = 0;
                            Position_MotorA = 0;
                            Position_MotorB = 0;
                            control_motor_A_RTR(0);
                            control_motor_B_RTR(0);
                            P_rotate_A = 0;
                            P_rotate_B = 0;
                            state_rotate = 0;
                            state_fb = "";
                            sg90.write(90);
                            mySerial.println(three);
                            vTaskDelay(100 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU          
                            state_auto = 0;
                        }
                    }
                    prev_e_A = e_A;
                    prev_e_B = e_B;
                    Serial.print("Current Position RTL: ");
                    Serial.print(Position_MotorA);
                    Serial.print(" Target Position RTL: ");
                    Serial.print(P_rotate_A);
                    Serial.print(" PWM: ");
                    Serial.println(u_A);
                    Serial.print("Current Position RTR: ");
                    Serial.print(Position_MotorB);
                    Serial.print(" Target Position RTR: ");
                    Serial.print(P_rotate_B);
                    Serial.print(" PWM: ");
                    Serial.println(u_B);
                }else{
                    Position_MotorA = 0;
                    Position_MotorB = 0;
                    control_motor_A_RTR(0);
                    control_motor_B_RTR(0);
                    state_auto = 0;
                    state_fb ="";
                    P_rotate_A = 0;
                    P_rotate_B = 0;
                    sg90.write(90);
                    mySerial.println(three);
                    vTaskDelay(100 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU          
                }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
    }
}
void Task_handler_PID_str(void *pvParameters){
    while(1){
        if(state_str == 1 && stateAuto == "True"){ 
            state_auto = 0;
            int P_str = ((distance*330*10)/204)*3;
            long currT_str_A = micros();
            long currT_str_B = micros();
            float deltaT_str_A = ((float)(currT_str_A - prevT_str_A)) / 1000000.0;
            float deltaT_str_B = ((float)(currT_str_B - prevT_str_B)) / 1000000.0;
            prevT_str_A = currT_str_A;
            prevT_str_B = currT_str_B;

            // Lấy giá trị hiện tại của Position_MotorA
            noInterrupts();
            int Position_MotorA_STR = Position_MotorA;
            int Position_MotorB_STR = Position_MotorB;
            interrupts();
            float e_str_A = Position_MotorA_STR - P_str;
            float e_str_B = Position_MotorB_STR - P_str;
            float dedt_str_A = (e_str_A- eprev_str_A) / deltaT_str_A;
            float dedt_str_B = (e_str_B- eprev_str_B) / deltaT_str_B;
            enitegral_str_A += e_str_A * deltaT_str_A;
            enitegral_str_B += e_str_B * deltaT_str_B;

            // Tính toán tín hiệu điều khiển PID
            float u_str_A = Kp_STR * e_str_A + Ki_STR * dedt_str_A + Kd_STR * enitegral_str_A;
            float u_str_B = Kp_STR * e_str_B + Ki_STR * dedt_str_B + Kd_STR * enitegral_str_B;
            u_str_A = (int)u_str_A;
            u_str_B = (int)u_str_B;
            if(u_str_A >= 255){
                u_str_A = 255;
            }else if(u_str_A <= -255){
                u_str_A = -255;
            }
            if(u_str_B >= 255){
                u_str_B = 255;
            }else if(u_str_B <= -255){
                u_str_B = -255;
            }
            // control_motor_A_str(u_str_B);
            control_motor_A_str(u_str_A);
            eprev_str_A = e_str_A;
            eprev_str_B = e_str_B;
            if(fabs(e_str_A) <= 20 || distanceCm >= 40){ //|| fabs(e_str_B) <= 20
                count_str++;
                if(count_str == 1){
                    state_auto = 0;
                    Position_MotorA = 0;
                    Position_MotorB = 0;
                    state_str = 0;
                    state_fb = "";
                    Position_MotorA_STR = 0;
                    Position_MotorB_STR = 0;
                    mySerial.println(one);  
                    vTaskDelay(100 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU          
                    control_motor_A_str(0);
                    control_motor_B_str(0);
                    count_str = 0;
                    state_pick = 0;
                    flag_Servo = 0;
                }
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
    }
}
void Task_distance_ultrasonic(void *pvParameters){
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    while(1){
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);
        duration = pulseIn(ECHO_PIN, HIGH, TIME_OUT);
        distanceCm = duration / 29.1 / 2;
        // if(distanceCm >= 40 && stateAuto == "True" && state_str == 1){
        //     Serial.println("Stop");
        //     control_motor_A_str(0);
        //     control_motor_B_str(0);
        //     Position_MotorA = 0;
        //     Position_MotorB = 0;
        //     mySerial.println(one);  
        //     state_str = 0;
        //     state_fb = "";
        //     state_pick = 0;
        //     flag_Servo = 0;
        //     vTaskDelay(100 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU          
        // }
        vTaskDelay(20 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
    }
}
//Function to handle Pick-ball action
void handlePickBalls_Control(void *pvParameters){
    int PwmValue = PwmDataPickBalls.toInt();
    PwmValue = (PwmValue*255)/100;
    if(statePick == "True" && stateAuto == "False"){
        digitalWrite(IN5, HIGH);
        digitalWrite(IN6, LOW);
        analogWrite(ENC,PwmValue);
        digitalWrite(IN7, HIGH);
        digitalWrite(IN8, LOW);
        analogWrite(END,PwmValue);
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
    }else if(statePick == "False" && stateAuto == "False"){
        digitalWrite(IN5, LOW);
        digitalWrite(IN6, LOW);
        analogWrite(END,0);

        digitalWrite(IN7, LOW);
        digitalWrite(IN8, LOW);
        analogWrite(END,0);
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
}
void handlePickBalls_Control_Auto(){
        if(stateAuto == "True"){
            digitalWrite(IN5, HIGH);
            digitalWrite(IN6, LOW);
            analogWrite(ENC,100);
            digitalWrite(IN7, HIGH);
            digitalWrite(IN8, LOW);
            analogWrite(END,100);
            vTaskDelay(10 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
        }else if(stateAuto == "False"){
            digitalWrite(IN5, LOW);
            digitalWrite(IN6, LOW);
            analogWrite(END,0);

            digitalWrite(IN7, LOW);
            digitalWrite(IN8, LOW);
            analogWrite(END,0);
            vTaskDelay(10 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
        }
    vTaskDelay(1 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
}

void Task_handler_manualControl(void *pvParameters){
    while(1){
    int PwmValue = PwmDataAction.toInt();
    PwmValue = (PwmValue*255*10)/100; 
    if (receivedData == "87")
    {
        // Control Motor A and B to fwd
        analogWrite(ENA,PwmValue);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENB,PwmValue);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    }
    else if (receivedData == "83")
    {
        // Control Motor A and B to back
        analogWrite(ENA, PwmValue);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENB,PwmValue);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    }
    else if (receivedData == "68")
    {
        // Control Motor A and B to left
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA,PwmValue);

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB,PwmValue);
    }
    else if (receivedData == "65")
    {
        // Control Motor A and B to right
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA,PwmValue);

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB,PwmValue);
    }
    else if(receivedData == "0"){
        // Control Motor A and B to stop
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        analogWrite(ENA,PwmValue);

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        analogWrite(ENB,PwmValue);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPU
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Độ trễ để tránh chiếm CPUs
}
void setup(){
    Serial.begin(9600);
    mySerial.begin(9600);  // Khởi tạo SoftwareSerial trong setup()
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    pinMode(IN5, OUTPUT);
    pinMode(IN6, OUTPUT);
    pinMode(IN7, OUTPUT);
    pinMode(IN8, OUTPUT);
    pinMode(ENC, OUTPUT);
    pinMode(END, OUTPUT);
    pinMode(EncoderA,INPUT_PULLUP);
    pinMode(EncoderB,INPUT_PULLUP);
    pinMode(EncoderC,INPUT_PULLUP);
    pinMode(EncoderD,INPUT_PULLUP);

    // STARTING SPIFFS
    if (!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }
    Serial.println("SPIFFS mounted successfully");
   
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Không thể cấu hình địa chỉ IP tĩnh!");
    } 
    // STARTING WIFI STA
    WiFi.begin(ssid, password);
    Serial.print("IS CONNECTING TO WIFI ");
    Serial.print(ssid);

    // WAITING TO CONNECT
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.print(".");
    }
    Serial.println();
    Serial.println("Kết nối WiFi thành công!");
    Serial.print("Địa chỉ IP: ");
    Serial.println(WiFi.localIP());

    // DEFINITION ROUTER
    // HANDLE favicon.ico
    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/favicon.ico", "image/x-icon"); });
    // ROUTER FOR HTML
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/index.html", "text/html"); });

    // ROUTER FOR CSS
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/style.css", "text/css"); });

    // * CONFIG ROUTER FOR POST METHOD *
    // ROUTER FOR POST METHOD /submit
    server.on("/submit", HTTP_POST, handleSubmit);
    // Route for requires Speed value
    server.on("/PwmAction", HTTP_POST, handlePwmAction);
    // Route for requires Speed value Pick_balls
    server.on("/PickPwm", HTTP_POST, handlePwmPickBalls);
    //Route cho yêu cầu POST /
    server.on("/PickState", HTTP_POST, handlePickBalls);
    //Route cho yêu cầu POST /
    server.on("/AutoState", HTTP_POST, handleAutoMode);

    // * CONFIG ROUTER FOR POST METHOD *
    // Route cho yêu cầu GET /getData
    server.on("/getData", HTTP_GET, handleGetData);
    server.on("/getValue", HTTP_GET, handleGetValue);

    // STARTING WEBSERVER
    server.begin();
    Serial.println("Web server đã bắt đầu.");

    xTaskCreate(Task_Handler_Uart_recieve, "Uart Task", 1024, NULL, 1, NULL);  // Tăng stack size
    xTaskCreate(Task_Handler_Uart_send, "Uart Task", 1024, NULL, 2, NULL);  // Tăng stack siz
    xTaskCreate(Task_handler_Servo, "Servo Task", 1024, NULL, 3, NULL);  // Tăng stack size
    xTaskCreate(Task_handler_PID, "PID RT Task", 4096, NULL, 4, NULL);  // Tăng stack size
    xTaskCreate(Task_distance_ultrasonic, "Manual Handler", 1024, NULL, 5, NULL);
    xTaskCreate(Task_handler_manualControl, "Manual Handler", 4096, NULL, 6, NULL);  // Tăng stack size
    xTaskCreate(Task_handler_count_balls, "Manual Handler", 1024, NULL, 7, NULL);  // Tăng stack size
    xTaskCreate(Task_handler_check_Servo, "Manual Handler", 1024, NULL, 8, NULL);  // Tăng stack size 
    xTaskCreate(Task_handler_send_uart_state, "Manual Handler", 1024, NULL, 9, NULL);
    xTaskCreate(Task_handler_PID_str, "PID Str Task", 4096, NULL, 10, NULL);  // Tăng stack size
    attachInterrupt(EncoderA, handlerEncoder_Left, CHANGE);
    attachInterrupt(EncoderC, handlerEncoder_Right, CHANGE);  
}
void loop(){
    handlePickBalls_Control_Auto();
    delay(100);
}
