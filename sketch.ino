// Wiring buttons and sensor

const uint8_t PWM_PIN = 5; // Pin where PWM signal will be generated
uint16_t ldrValue = 0; // Value read on pin A0

const uint8_t pinRS = 4;
const uint8_t pinEN = 3;
const uint8_t pinD4 = 2;
const uint8_t pinD5 = 1;
const uint8_t pinD6 = 0;
const uint8_t pinD7 = 7;

const uint8_t startStopButton = 2;
volatile bool startStopButtonPressed = false;
volatile bool startStopButtonPressedLong = false;
volatile unsigned long buttonPressStartTime = 0; // Variable to store the start time of pressing the push button

const uint8_t movementPin = 3;
volatile unsigned long transitionTime = 0; // Time to complete one revolution
volatile bool transitionDetected = false;

volatile long seconds = 0; // Initialization of seconds counter
volatile long milliseconds = 0; // Initialization of milliseconds counter

// Variables for calculating values
float lastRevolutionStartTime = 0; // Time when the last revolution started
volatile unsigned int nRevolution = 0; // Revolution counter
volatile float distance = 0; // Total recorded distance
volatile unsigned long currentSpeed = 0; // Instantaneous speed detected
volatile float T = 0; // Time elapsed since the last pulse
volatile float T1 = 0; // Time elapsed since the last pulse regardless of the LDR sensor operation

const float bicycleWheelCircumference = 2.1206; // Circumference of the bike wheel expressed in meters

void setup() {

  // External interrupt configuration on INT0 (pin 2)
  EICRA |= (1 << ISC00); // External interrupt on state change
  EIMSK |= (1 << INT0); // Enable external interrupt on INT0 (pin 2)

  // External interrupt configuration on INT1 (pin 3)
  EICRA |= (1 << ISC10); // External interrupt on state change
  EIMSK |= (1 << INT1); // Enable external interrupt on INT1 (pin 3)

  // Timer1 settings to generate an interrupt every second
  cli(); // Disable interrupts during configuration
  TCCR1A = 0; // Clear timer control registers
  TCCR1B = 0;
  TCNT1 = 0; // Initialize timer count to 0
  OCR1A = 15624; // Set comparison value to achieve one second
  TCCR1B |= (1 << WGM12); // Set timer in comparison mode with OCR1A
  TCCR1B |= (1 << CS12) | (1 << CS10); // Set prescaler to 1024
  TIMSK1 |= (1 << OCIE1A); // Enable interrupt for comparison with OCR1A

  // Set up timer2 to generate an interrupt every millisecond
  TCCR2A = 0; // Clear control registers A
  TCCR2B = 0; // Clear control registers B
  TCNT2 = 0; // Clear timer2 count
  OCR2A = 249; // Set comparison value to generate an interrupt every millisecond
  TCCR2A |= (1 << WGM21); // Set timer in comparison mode with OCR2A
  TCCR2B |= (1 << CS22); // Set prescaler to 64
  TIMSK2 |= (1 << OCIE2A); // Enable interrupt for comparison with OCR2A

  // Configuration of the timer 0 overflow interrupt
  TIMSK0 |= (1 << TOIE0); // Enable timer 0 overflow interrupt
  // Configuration of timer 0
  TCCR0A |= (1 << WGM00) | (1 << WGM01); // Fast PWM mode
  TCCR0B |= (1 << CS02); // Prescaler of 256 (clock frequency of 16MHz / 256)
  OCR0A = 255; // Maximum value of the counter (PWM signal period)

  sei(); // Enable interrupts after configuration

  DDRD |= (1 << PWM_PIN); // Set pin 5 that generates the PWM signal as output

  TCCR0A = _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); // PWM setup
  TCCR0B = _BV(CS01); // Set clock frequency to 64

  // Enable AVCC reference voltage (5V)
  ADMUX |= (1 << REFS0);
  // Set prescaler to 128 to achieve a sampling frequency of 125 kHz
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  // Enable ADC converter
  ADCSRA |= (1 << ADEN);

  // Initialization of display pins as output
  DDRB |= (1 << pinRS);
  DDRB |= (1 << pinEN);
  DDRB |= (1 << pinD4);
  DDRB |= (1 << pinD5);
  DDRB |= (1 << pinD6);
  DDRD |= (1 << pinD7);

  // Enable the Movement pushButton as input-pullup
  DDRD &= ~(1 << movementPin);
  // Set internal pull-up
  PORTD |= (1 << movementPin);

  // Enable Start pushButton pins as input-pullup
  // Set the pin as input
  DDRD &= ~(1 << startStopButton);
  // Set internal pull-up
  PORTD |= (1 << startStopButton);

  // Initialization of the LCD display
  sendCommand(0x33); // 8-bit initialization sequence
  sendCommand(0x32); // 4-bit initialization sequence
  sendCommand(0x28); // 2-line mode, 5x8 character
  // Enable the LCD display
  sendCommand(0x0C);
  // Clear the LCD display
  sendCommand(0x01);
  // Set the initial contrast of the display
  sendCommand(0x0C); // Display on, cursor off, blinking off
  sendCommand(0x06); // Increment cursor
  sendCommand(0x01); // Clear the display

  // Set the cursor to the first row, first character
  sendCommand(0x80);
  // Write the message
  sendText("Push the button");
  // Set the cursor to the second row, first character
  sendCommand(0xC4);
  sendText("to start");
}

void loop() {
    // Start ADC conversion for the LDR sensor
  ADCSRA |= (1 << ADSC);
  // Wait for the end of ADC conversion
  while (ADCSRA & (1 << ADSC));
  // Read the value converted by the LDR sensor
  ldrValue = ADC;

  OCR0B = map(ldrValue, 0, 1023, 255, 0); // Convert the read value to PWM and write it to the OCR0B register

  if (startStopButtonPressed) {

    if (transitionDetected) {
      T = milliseconds - lastRevolutionStartTime; // Compute the time to complete one revolution
      distance = (nRevolution * bicycleWheelCircumference) / 1000; // Calculate the distance
      currentSpeed = (bicycleWheelCircumference / T) * 3600; // Calculate the speed
      transitionDetected = false; // Reset the boolean value to false
    }

    T1 = milliseconds - lastRevolutionStartTime; // Read the time elapsed since the last pulse
    // even if a transition was not detected
    updateLCD(); // Update the display with time information
  }
  if (startStopButtonPressedLong) {
    // Reset Arduino
    asm("jmp 0");
    startStopButtonPressedLong = false; // Reset the value to false
  }
}

// Interrupt service routine for the LDR sensor
ISR(INT1_vect) {
  if ((!(PIND & (1 << movementPin)))) { // Button pressed
    transitionDetected = true;
    nRevolution++;
  } else //if (PIND & (1 << movementPin))
  { // Button released
    lastRevolutionStartTime = milliseconds;
  }
}

// Interrupt service routine for the startStopButton
ISR(INT0_vect) {
  if (!(PIND & (1 << startStopButton))) { // Button pressed
    buttonPressStartTime = milliseconds;
  } else { // Button released

    unsigned long buttonPressDuration = milliseconds - buttonPressStartTime; // Duration of button press

    if (buttonPressDuration >= 1000) { // If the press is >= 1 second, detect a long press
      startStopButtonPressedLong = true;
    } else if (buttonPressDuration > 0 && buttonPressDuration < 1000) { // If it's less than a second but still pressed
      startStopButtonPressed = true;
    }
  }
}

// Interrupt service routine for the counter
ISR(TIMER1_COMPA_vect) {
  if (startStopButtonPressed) {
    seconds++;
  } // Start the seconds counter only if the program has been started
}

// Timer2 interrupt for milliseconds counting
ISR(TIMER2_COMPA_vect) {
  milliseconds++;
}

void sendCommand(byte command) {
  PORTB &= ~_BV(pinRS); // Set pin RS low for commands
  sendData(command);
}

void sendNum(volatile float num) {
  char buffer[10];
  dtostrf(num, 1, 0, buffer); // Convert the number to a string format with 5 total digits and 2 decimals
  sendText(buffer);
}

void sendNumFloat(volatile float num) {
  char buffer[10];
  dtostrf(num, 5, 2, buffer); // Convert the number to a string format with 1 total digit
  sendText(buffer);
}

void sendText(const char * text) {
  PORTB |= _BV(pinRS); // Set pin RS high for data
  while (*text) {
    sendData(*text);
    text++;
  }
}

void sendData(byte data) {
  PORTB &= ~_BV(3); // EN pin low -> Turn on the display

  // Sending the 4 most significant bits, first nibble
  // Set pin D4 with the least significant bit of 'data'
  PORTB = (PORTB & ~_BV(pinD4)) | (((data >> 4) & 1) << 2);
  // Set pin D5 with the second least significant bit of 'data'
  PORTB = (PORTB & ~_BV(pinD5)) | (((data >> 5) & 1) << 1);
  // Set pin D6 with the third least significant bit of 'data'
  PORTB = (PORTB & ~_BV(pinD6)) | (((data >> 6) & 1) << 0);
  // Set pin D7 with the fourth least significant bit of 'data'
  PORTD = (PORTD & ~_BV(pinD7)) | (((data >> 7) & 1) << 7);

  PORTB |= _BV(pinEN); // EN pin high -> Turn off the display
  PORTB &= ~_BV(pinEN); // EN pin low -> Turn on the display

  // Sending the 4 least significant bits, second nibble
  // Set pin D4 with the least significant bit of 'data'
  PORTB = (PORTB & ~_BV(pinD4)) | (((data >> 0) & 1) << 2);
  // Set pin D5 with the second least significant bit of 'data'
  PORTB = (PORTB & ~_BV(pinD5)) | (((data >> 1) & 1) << 1);
  // Set pin D6 with the third least significant bit of 'data'
  PORTB = (PORTB & ~_BV(pinD6)) | (((data >> 2) & 1) << 0);
  // Set pin D7 with the fourth least significant bit of 'data'
  PORTD = (PORTD & ~_BV(pinD7)) | (((data >> 3) & 1) << 7);

  PORTB |= _BV(3); // EN pin high -> Turn off the display
}

void updateLCD() {
  // Set the cursor to the first row, first character
  sendCommand(0x80);
  // Write the message
  sendText("Speed:    ");

  if (T1 > 0 && T1 < 10000) {
    sendNum(currentSpeed);
  } else //if (T1 > 10000)
  {
    sendNum(0);
  } // If the time elapsed since the last pulse is >=10 seconds, then
  // the user is stationary, and the instantaneous speed is set to 0

  sendText("km/h");

  // Calculate hours, minutes, and seconds
  int hours = seconds / 3600;
  int minutes = (seconds / 60) % 60;
  int second = seconds % 60;

  // Set the cursor to the second row, first character
  sendCommand(0xC0);
  //sendText("Time: ");
  sendNum(hours);
  sendText(":");
  if (minutes < 10) {
    sendNum(0); // Add leading zero if minutes are less than 10
  }
  sendNum(minutes);
  sendText(":");
  if (second < 10) {
    sendNum(0); // Add leading zero if seconds are less than 10
  }
  sendNum(second);

  sendText("  "); // Add a space

  sendNumFloat(distance);
  sendText("km");
}
