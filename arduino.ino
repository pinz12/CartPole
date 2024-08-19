const int analogPin = A0;  // Pin to angle sensor 
const int dirPin = 2;
const int stepPin = 3;
const int switchPin = 4;  // Pin to switch

unsigned long previousMillis = 0;
const long interval = 50;

bool switchState = LOW;      // Current switching status
bool lastSwitchState = LOW;  // Previous switching status
bool loopEnabled = false;

bool isTestModeEnabled = true;

double previousAngle = 0;
unsigned long previousTime = 0;
double totalDisplacement = 0;

struct State {
  double currentAngle;
  double angularVelocity;
  double displacement;
};

// void getAngel() {
//   int analogInputVal = analogRead(A0) - 6;
//   double currentAngle = fmod((double)analogInputVal, 102.4) * 3.6 - 180;
//   String output = String(currentAngle) + "°";
//   Serial.print("Angle: ");
//   Serial.println(output);
//   delay(100);
// }

// void getAngelAndAngularVelocity() {
//   int analogInputVal = analogRead(A0) - 6;
//   double currentAngle = fmod((double)analogInputVal, 102.4) * 3.6 - 180;
//   unsigned long currentTime = millis();
//   double deltaTime = (currentTime - previousTime) / 1000.0;
//   double angularVelocity = 0;
//   if (deltaTime > 0) {
//     angularVelocity = (currentAngle - previousAngle) / deltaTime;
//   }
//   Serial.print("Angle: ");
//   Serial.print(currentAngle);
//   Serial.print("°, Angular Velocity: ");
//   Serial.print(angularVelocity);
//   Serial.println("°/s");
//   previousAngle = currentAngle;
//   previousTime = currentTime;
//   delay(100);
// }

struct State getState() {
  Serial.print("States: ");
  struct State state;

  int analogInputVal = analogRead(A0);
  double a = (analogInputVal % 102) * (360.0 / 102.4) - 200;
  if (a < 0) {
    a += 360;
  }
  state.currentAngle = a;

  unsigned long currentTime = millis();
  double deltaTime = (currentTime - previousTime) / 1000.0;

  state.angularVelocity = 0;
  if (deltaTime > 0) {
    state.angularVelocity = (state.currentAngle - previousAngle) / deltaTime;
  }

  state.displacement = totalDisplacement;
  previousAngle = state.currentAngle;
  previousTime = currentTime;

  return state;
}

// dir: true to the left, false to the right
void move(int step, int delay, bool dir) {
  for (int i = 0; i < step; i++) {
    if (dir) {
      digitalWrite(dirPin, HIGH);
    } else {
      digitalWrite(dirPin, LOW);
    }
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delay);
  }
}

// 0: one step to the left
// 1: one step to the right
void action(int a) {
  switch (a) {
    case 0:
      move(88, 800, true);
      totalDisplacement -= 88;
      break;
    case 1:
      move(88, 800, false);
      totalDisplacement -= 88;
      break;
  }
}

void disableTestMode() {
  isTestModeEnabled = false;
  Serial.println("Test mode is disabled");
}

void enableTestMode() {
  isTestModeEnabled = true;
  Serial.println("Test mode is enabled");
}

void work() {
  // if (Serial.available() > 0) {
  //   int actionSignal = Serial.parseInt();
  //   Serial.print("Action of Agent: ");
  //   Serial.println(actionSignal);
  //   if(!isTestModeEnabled){
  //     action(actionSignal);
  //   }
  // }

  if (Serial.available() > 0) {
    String receivedString = Serial.readStringUntil('\n');
    if (receivedString.startsWith("Action: ")) {
      int actionSignal = receivedString.substring(15).toInt();
      if (!isTestModeEnabled) {
        action(actionSignal);
        Serial.println("Motor moved");
      }
    }
  }

  struct State state = getState();
  Serial.print((int)state.currentAngle);
  Serial.print(" ");
  Serial.print((int)state.angularVelocity);
  Serial.print(" ");
  Serial.println((int)state.displacement);
}

void setup() {
  Serial.begin(9600);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP);
  disableTestMode();
  int analogInputVal = analogRead(A0);
  double a = (analogInputVal % 102) * (360.0 / 102.4) - 200;
  if (a < 0) {
    a += 360;
  }
  previousAngle = a;
}

void loop() {

  switchState = digitalRead(switchPin);

  if (switchState == LOW && lastSwitchState == HIGH) {
    Serial.print("Switch pressed");
    loopEnabled = !loopEnabled;
    if (loopEnabled) {
      Serial.println(", loop on");
    } else {
      Serial.println(", loop off");
    }
  }

  lastSwitchState = switchState;

  unsigned long currentMillis = millis();
  if (loopEnabled && currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    work();
  }
}
