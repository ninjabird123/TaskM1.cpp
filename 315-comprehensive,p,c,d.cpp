// ============================================================
// SIT315 M1.T1P - QP4
// Multi-Sensor System using Pin-Change Interrupts + Timer
//
// Sensors:
// D8  = Sensor 1
// D9  = Sensor 2
// D10 = Sensor 3
//
// Actuator:
// D4 = LED
//
// QP4 REQUIREMENTS:
// - 3 digital sensors
// - Pin-change interrupts (PCINT)
// - Shared PCINT0 vector
// - Identification of which sensor changed
// - Sensor state management
// - LED ON when at least 2 sensors are active
// - Debounce handling in main loop
// - Timer1 periodic interrupt
// - Timer ISR is short and non-blocking
// - Event-based PCINT logic separated from
//   time-based Timer logic
// ============================================================

const int sensor1Pin = 8;   // D8 / PB0 / PCINT0
const int sensor2Pin = 9;   // D9 / PB1 / PCINT1
const int sensor3Pin = 10;  // D10 / PB2 / PCINT2

const int ledPin = 4;


// ============================================================
// SENSOR STATES
// ============================================================

volatile bool sensor1Active = false;
volatile bool sensor2Active = false;
volatile bool sensor3Active = false;


// ============================================================
// PCINT CHANGE MASK
//
// bit 0 = D8 changed
// bit 1 = D9 changed
// bit 2 = D10 changed
// ============================================================

volatile uint8_t changedMask = 0;


// ============================================================
// PREVIOUS PORTB STATE
// ============================================================

volatile uint8_t lastPortB;


// ============================================================
// DEBOUNCE
// ============================================================

unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long lastDebounceTime3 = 0;

const unsigned long debounceDelay = 50;


// ============================================================
// TIMER EVENT FLAG
//
// Timer ISR only sets this flag.
// The main loop performs the actual periodic task.
//
// This keeps the ISR short and non-blocking.
// ============================================================

volatile bool timerEvent = false;


// ============================================================
// PCINT0 INTERRUPT SERVICE ROUTINE
//
// D8, D9 and D10 belong to PORTB.
//
// All three therefore share PCINT0_vect.
//
// ISR only detects which pins changed.
// No Serial.
// No delay().
// No blocking code.
// ============================================================

ISR(PCINT0_vect)
{
  uint8_t currentPortB = PINB;

  // Detect which PORTB bits changed
  uint8_t changed = currentPortB ^ lastPortB;

  // Save current PORTB state
  lastPortB = currentPortB;

  // Only monitor PB0, PB1 and PB2
  changedMask |= (changed & 0x07);
}


// ============================================================
// TIMER1 INTERRUPT SERVICE ROUTINE
//
// Timer1 generates a periodic interrupt every 1 second.
//
// IMPORTANT:
// The ISR does NOT print Serial messages.
// It only sets a flag.
//
// The main loop handles the actual periodic task.
// ============================================================

ISR(TIMER1_COMPA_vect)
{
  timerEvent = true;
}


// ============================================================
// CONFIGURE PIN-CHANGE INTERRUPTS
// ============================================================

void setupPCINT()
{
  // Enable PCINT group 0
  PCICR |= (1 << PCIE0);

  // Enable PCINT0 = D8
  PCMSK0 |= (1 << PCINT0);

  // Enable PCINT1 = D9
  PCMSK0 |= (1 << PCINT1);

  // Enable PCINT2 = D10
  PCMSK0 |= (1 << PCINT2);
}


// ============================================================
// CONFIGURE TIMER1
//
// Arduino Uno clock = 16 MHz
//
// Prescaler = 1024
//
// Timer frequency:
// 16,000,000 / 1024 = 15,625 Hz
//
// For 1 second:
// 15,625 - 1 = 15,624
//
// Therefore OCR1A = 15624
// ============================================================

void setupTimer1()
{
  // Stop Timer1 during configuration
  TCCR1A = 0;
  TCCR1B = 0;

  // Reset Timer1 counter
  TCNT1 = 0;

  // Compare value for approximately 1 second
  OCR1A = 15624;

  // CTC mode
  TCCR1B |= (1 << WGM12);

  // Prescaler = 1024
  TCCR1B |= (1 << CS12);
  TCCR1B |= (1 << CS10);

  // Enable Timer1 Compare Match A interrupt
  TIMSK1 |= (1 << OCIE1A);
}


// ============================================================
// READ SENSOR STATE
//
// INPUT_PULLUP means:
//
// LOW  = button pressed  = ACTIVE
// HIGH = button released = INACTIVE
// ============================================================

bool readSensorState(int pin)
{
  return digitalRead(pin) == LOW;
}


// ============================================================
// REPORT SENSOR CHANGES
// ============================================================

void reportSensorChanges(uint8_t changes)
{
  if (changes & 0x01)
  {
    Serial.print("D8 Sensor 1 changed -> ");
    Serial.println(sensor1Active ? "ACTIVE" : "INACTIVE");
  }

  if (changes & 0x02)
  {
    Serial.print("D9 Sensor 2 changed -> ");
    Serial.println(sensor2Active ? "ACTIVE" : "INACTIVE");
  }

  if (changes & 0x04)
  {
    Serial.print("D10 Sensor 3 changed -> ");
    Serial.println(sensor3Active ? "ACTIVE" : "INACTIVE");
  }
}


// ============================================================
// PROCESS SYSTEM STATE
//
// LED ON when at least TWO sensors are active.
// ============================================================

void processSystemState()
{
  // Safely copy volatile states
  noInterrupts();

  bool s1 = sensor1Active;
  bool s2 = sensor2Active;
  bool s3 = sensor3Active;

  interrupts();

  int activeSensors = s1 + s2 + s3;

  if (activeSensors >= 2)
  {
    digitalWrite(ledPin, HIGH);

    Serial.print("Active sensors: ");
    Serial.print(activeSensors);
    Serial.println(" -> LED ON");
  }
  else
  {
    digitalWrite(ledPin, LOW);

    Serial.print("Active sensors: ");
    Serial.print(activeSensors);
    Serial.println(" -> LED OFF");
  }
}


// ============================================================
// PROCESS TIMER EVENT
//
// This is the TIME-BASED task.
//
// The Timer ISR only sets timerEvent.
// The main loop performs the Serial output.
// ============================================================

void processTimerEvent()
{
  bool event = false;

  noInterrupts();

  if (timerEvent)
  {
    timerEvent = false;
    event = true;
  }

  interrupts();

  if (event)
  {
    Serial.println("TIMER: 1 second periodic event");
  }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  // ----------------------------------------------------------
  // Configure sensors
  // ----------------------------------------------------------

  pinMode(sensor1Pin, INPUT_PULLUP);
  pinMode(sensor2Pin, INPUT_PULLUP);
  pinMode(sensor3Pin, INPUT_PULLUP);


  // ----------------------------------------------------------
  // Configure LED
  // ----------------------------------------------------------

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);


  // ----------------------------------------------------------
  // Start Serial
  // ----------------------------------------------------------

  Serial.begin(9600);


  // ----------------------------------------------------------
  // Read initial PORTB state
  // BEFORE enabling interrupts
  // ----------------------------------------------------------

  lastPortB = PINB;


  // ----------------------------------------------------------
  // Initialise sensor states
  // ----------------------------------------------------------

  sensor1Active = readSensorState(sensor1Pin);
  sensor2Active = readSensorState(sensor2Pin);
  sensor3Active = readSensorState(sensor3Pin);


  // ----------------------------------------------------------
  // Configure PCINT
  // ----------------------------------------------------------

  setupPCINT();


  // ----------------------------------------------------------
  // Configure Timer1
  // ----------------------------------------------------------

  setupTimer1();


  // ----------------------------------------------------------
  // Startup message
  // ----------------------------------------------------------

  Serial.println("==============================");
  Serial.println("QP4 PCINT + TIMER SYSTEM");
  Serial.println("Monitoring D8, D9 and D10");
  Serial.println("LED ON when >= 2 sensors active");
  Serial.println("Timer event every 1 second");
  Serial.println("==============================");
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  // ==========================================================
  // 1. PROCESS TIMER EVENT
  // ==========================================================

  processTimerEvent();


  // ==========================================================
  // 2. GET PCINT CHANGES
  // ==========================================================

  uint8_t changes = 0;

  noInterrupts();

  if (changedMask != 0)
  {
    changes = changedMask;
    changedMask = 0;
  }

  interrupts();


  // ==========================================================
  // 3. PROCESS D8
  // ==========================================================

  if (changes & 0x01)
  {
    unsigned long now = millis();

    if (now - lastDebounceTime1 >= debounceDelay)
    {
      lastDebounceTime1 = now;

      sensor1Active = readSensorState(sensor1Pin);

      reportSensorChanges(0x01);

      processSystemState();
    }
  }


  // ==========================================================
  // 4. PROCESS D9
  // ==========================================================

  if (changes & 0x02)
  {
    unsigned long now = millis();

    if (now - lastDebounceTime2 >= debounceDelay)
    {
      lastDebounceTime2 = now;

      sensor2Active = readSensorState(sensor2Pin);

      reportSensorChanges(0x02);

      processSystemState();
    }
  }


  // ==========================================================
  // 5. PROCESS D10
  // ==========================================================

  if (changes & 0x04)
  {
    unsigned long now = millis();

    if (now - lastDebounceTime3 >= debounceDelay)
    {
      lastDebounceTime3 = now;

      sensor3Active = readSensorState(sensor3Pin);

      reportSensorChanges(0x04);

      processSystemState();
    }
  }
}