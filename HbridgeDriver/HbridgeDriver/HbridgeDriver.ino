
int Motor[4][2] = //two dimensional array
{
{2 , 4},   //input pin to control Motor1 (front right)--> Motor[0][0]=4, Motor[0][1]=5
{6 , 7},   //input pin to control Motor2 (back right)--> Motor[1][0]=6, Motor[1][1]=7
{8 , 10},   //input pin to control Motor3 (front left)--> Motor[2][0]=8, Motor[2][1]=9
{12, 13},  //input pin to control Motor4 (back left)--> Motor[3][0]=10, Motor[3][1] = 11
};

#define EN1  3
#define EN2  5
#define EN3  9
#define EN4  11

#define STOP  'S'
#define FORWARD  'F'
#define BACKWARD  'B'
#define RIGHT 'R'
#define LEFT 'L'


const char SoP = 'C';
const char EoP = 'E';

const char nullTerminator = '\0';
unsigned char inByte;
#define MESSAGE_MAX_SIZE 5
char message[MESSAGE_MAX_SIZE];
char command;

int SPEED = 100;
void stop_bot();
void forwards();
void backwards();
void right();
void left();
void change_speed(int new_speed);
bool parsePacket();

//This will run only one time.
void setup(){

   Serial.begin(9600);
  Serial.println("START");

  pinMode(Motor[0][0], OUTPUT);  
  pinMode(Motor[0][1], OUTPUT);
  pinMode(Motor[1][0], OUTPUT);  
  pinMode(Motor[1][1], OUTPUT); 
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);

 pinMode(EN3, OUTPUT);
 pinMode(EN4, OUTPUT);
 pinMode(Motor[2][0], OUTPUT);  
 pinMode(Motor[2][1], OUTPUT);
 pinMode(Motor[3][0], OUTPUT);  
 pinMode(Motor[3][1], OUTPUT); 

}


void loop(){
   /// 1. get legal message
    if (!parsePacket())
        return;

    /// 2. action, for now we only use option 1
    if (message[0] == '1') {
        // Move command
        command = message[1];
        moveRobot('S');
        delay(1000);
        moveRobot(command);
    }
    else if (message[0] == '2') {
        // Display Read
        // ...
    }
    else if (message[0] == '3') {
        // Distance Read
        // ...
    }
    else if (message[0] == '4') {
        // Display Write
        Serial.println(message);
        return;
    }
    else {
        Serial.println("ERROR: unknown message");
        return;
    }

}

bool parsePacket() {
    /// step 1. get SoP
    while (Serial.available() < 1) {};
    inByte = Serial.read();
    if (inByte != SoP) {
        Serial.print("ERROR: Expected SOP, got: ");
        Serial.write((byte)inByte);
        Serial.print("\n");
        return false;
    }

    /// step 2. get message length
    while (Serial.available() < 1) {};
    inByte = Serial.read();
    if (inByte == EoP || inByte == SoP) {
        Serial.println("ERROR: SoP/EoP in length field");
        return false;
    }
    int message_size = inByte - '0';
    if (message_size > MESSAGE_MAX_SIZE || message_size < 0) {
        Serial.println("ERROR: Packet Length out of range");
        return false;
    }

    /// step 3. get message
    for (int i = 0; i < message_size; i++) {
        while (Serial.available() < 1) {};
        inByte = Serial.read();
        if ((inByte == EoP || inByte == SoP)) {
            Serial.println("ERROR: SoP/EoP in command field");
            return false;
        }
        message[i] = (char)inByte;
    }
    message[message_size] = nullTerminator;

    /// step 4. get EoP
    while (Serial.available() < 1) {};
    inByte = Serial.read();
    if (inByte != EoP) {
        Serial.println("EoP not found");
        return false;
    } else {
        return true;
    }
}

void moveRobot(char command) {
     switch(command) {
        case FORWARD:
            Serial.println("FORWARD");
            forwards();
            break;
        case BACKWARD:
            Serial.println("BACKWARD");
            backwards();
            break;
        case STOP:
            Serial.println("STOP");
            stop_bot();
            break;
        case RIGHT:
            Serial.println("RIGHT");
            right();
            break;
        case LEFT:
            Serial.println("LEFT");
            left();
            break;
        default:
            Serial.println("ERROR: Unknown command in legal packet");
            break;
    }
}

void change_speed(int new_speed) {
  SPEED = new_speed;  
}

void stop_bot() {
  motor_run(0, EN1, STOP);
  motor_run(1, EN2, STOP);
  motor_run(2, EN3, STOP);
  motor_run(3, EN4, STOP);
}

void forwards() {
  motor_run(0, EN1, FORWARD); 
  motor_run(1, EN2, FORWARD); 
  motor_run(2, EN3, FORWARD); 
  motor_run(3, EN4, FORWARD); 
}

void backwards(){
  motor_run(0, EN1, BACKWARD);
  motor_run(1, EN2, BACKWARD);
  motor_run(2, EN3, BACKWARD);
  motor_run(3, EN4, BACKWARD); 
}

void right() {
  motor_run(0, EN1, BACKWARD); 
  motor_run(1, EN2, BACKWARD);    
  motor_run(2, EN3, FORWARD);    
  motor_run(3, EN4, FORWARD);    
}

void left() {
  motor_run(0, EN1, FORWARD);    
  motor_run(1, EN2, FORWARD);    
  motor_run(2, EN3, BACKWARD);    
  motor_run(3, EN4, BACKWARD);
}

void motor_run(int motor, int enable, int movement) {
  switch (movement) {
    case FORWARD:  
      analogWrite(enable, SPEED);
      digitalWrite(Motor[motor][0], HIGH);
      digitalWrite(Motor[motor][1], LOW);
      break;
    case BACKWARD:   
      analogWrite(enable, SPEED);
      digitalWrite(Motor[motor][0], LOW);
      digitalWrite(Motor[motor][1], HIGH);
      break; 
    case STOP:  
      digitalWrite(Motor[motor][0], LOW);
      digitalWrite(Motor[motor][1], LOW);
      break;   
    }   
  }     

