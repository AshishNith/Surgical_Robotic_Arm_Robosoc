// --- Stepper Motor Pins ---
#define stepPin 3
#define dirPin 2
int ms1 = 10;
int ms2 = 9;
int ms3 = 8;

// --- Rotary Encoder Pins ---
#define encoderCLK 4
#define encoderDT 5
#define encoderSW 6

// --- Stepper Mode Configuration ---
typedef struct {
  const char* mode;
  int multiplier;
  int ms1;
  int ms2;
  int ms3;
} StepperConfig;

StepperConfig stepperModes[] = {
  { "full",      1, LOW,  LOW,  LOW },
  { "half",      2, HIGH, LOW,  LOW },
  { "quarter",   4, LOW,  HIGH, LOW },
  { "eighth",    8, HIGH, HIGH, LOW },
  { "sixteenth", 16, HIGH, HIGH, HIGH }
};

int stepModeIndex = 0;
int baseStepsPerRev = 200;
int effectiveStepsPerRev = 200;

// --- Encoder State Tracking ---
int lastCLK = HIGH;
bool lastSWState = HIGH;
unsigned long lastDebounceTime = 0;

void setup() {
  // Stepper pins
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(ms1, OUTPUT);
  pinMode(ms2, OUTPUT);
  pinMode(ms3, OUTPUT);

  // Encoder pins
  pinMode(encoderCLK, INPUT_PULLUP);
  pinMode(encoderDT, INPUT_PULLUP);
  pinMode(encoderSW, INPUT_PULLUP);

  Serial.begin(9600);
  updateStepperMode();
}

void loop() {
  handleEncoderRotation();
  handleButtonPress();
}

// --- Handle Rotation to Move Stepper ---
void handleEncoderRotation() {
  int currentCLK = digitalRead(encoderCLK);
  if (currentCLK != lastCLK) {
    int currentDT = digitalRead(encoderDT);
    if (currentDT != currentCLK) {
      digitalWrite(dirPin, HIGH); // CW
    } else {
      digitalWrite(dirPin, LOW);  // CCW
    }
    stepOnce();
  }
  lastCLK = currentCLK;
}

// --- Handle Button Press to Change Step Mode ---
void handleButtonPress() {
  bool currentSW = digitalRead(encoderSW);
  if (currentSW == LOW && lastSWState == HIGH && (millis() - lastDebounceTime > 200)) {
    stepModeIndex = (stepModeIndex + 1) % (sizeof(stepperModes) / sizeof(stepperModes[0]));
    updateStepperMode();
    lastDebounceTime = millis();
  }
  lastSWState = currentSW;
}

// --- Perform One Step ---
void stepOnce() {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(500);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(500);
}

// --- Update Stepper Mode Pins & Settings ---
void updateStepperMode() {
  StepperConfig mode = stepperModes[stepModeIndex];
  digitalWrite(ms1, mode.ms1);
  digitalWrite(ms2, mode.ms2);
  digitalWrite(ms3, mode.ms3);
  effectiveStepsPerRev = baseStepsPerRev * mode.multiplier;

  Serial.print("Mode changed to: ");
  Serial.print(mode.mode);
  Serial.print(" | Steps/rev: ");
  Serial.println(effectiveStepsPerRev);
}