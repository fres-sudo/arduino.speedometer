//Wiring buttons and sensor

const uint8_t PWM_PIN = 5; //pin su cui verrà generato un segnale PWM
uint16_t ldrValue = 0; //valore letto sul pin A0

const uint8_t pinRS = 4;
const uint8_t pinEN = 3;
const uint8_t pinD4 = 2;
const uint8_t pinD5 = 1;
const uint8_t pinD6 = 0;
const uint8_t pinD7 = 7;

const uint8_t startStopButton = 2;
volatile bool startStopButtonPressed = false;
volatile bool startStopButtonPressedLong = false;
volatile unsigned long buttonPressStartTime = 0; // Variabile per memorizzare il tempo di inizio del premere il push button

const uint8_t movementPin = 3;
volatile unsigned long transitionTime = 0; // tempo per fare una rivoluzione
volatile bool transitionDetected = false;

volatile long seconds = 0; //inizializzazione del counter di secondi
volatile long milliseconds = 0; //inizializzazione del counter di millisecondi

//variabili per il calcolo dei valori
float lastRevolutionStartTime = 0; //tempo in cui è iniziata l'ultima rivoluzione
volatile unsigned int nRevolution = 0; // counter di rivoluzioni
volatile float distance = 0; //distanza totale registrata
volatile unsigned long currentSpeed = 0; //velocità istantanea rilevata
volatile float T = 0; // Tempo trascorso dall'ultimo impulso
volatile float T1 = 0; // Tempo trascorso dall'ultimo impulso a prescindere dal funzionamento del sensore LDR

const float bicycleWheelCircumference = 2.1206; //circonferenza della ruota della bici espressa in metri

void setup() {

  // Configurazione interrupt esterno su INT0 (pin 2)
  EICRA |= (1 << ISC00); // Interruzione esterna su cambio di stato
  EIMSK |= (1 << INT0); // Abilita interrupt esterno su INT0 (pin 2)

  // Configurazione interrupt esterno su INT1 (pin 3)
  EICRA |= (1 << ISC10); // Interruzione esterna su cambio di stato
  EIMSK |= (1 << INT1); // Abilita interrupt esterno su INT1 (pin 3)

  // Impostazioni del timer1 per generare un interrupt ogni secondo
  cli(); // Disabilita gli interrupt durante la configurazione
  TCCR1A = 0; // Azzera i registri di controllo del timer
  TCCR1B = 0;
  TCNT1 = 0; // Inizializza il conteggio del timer a 0
  OCR1A = 15624; // Imposta il valore di confronto per ottenere un secondo
  TCCR1B |= (1 << WGM12); // Imposta il timer in modalità di confronto con OCR1A
  TCCR1B |= (1 << CS12) | (1 << CS10); // Imposta il prescaler a 1024
  TIMSK1 |= (1 << OCIE1A); // Abilita l'interrupt per il confronto con OCR1A

  // Imposta il timer2 per generare un interrupt ogni millisecondo
  TCCR2A = 0; // Azzera il registri di controllo A
  TCCR2B = 0; // Azzera il registri di controllo B
  TCNT2 = 0; // Azzera il conteggio del timer2
  OCR2A = 249; // Imposta il valore di confronto per generare un interrupt ogni millisecondo
  TCCR2A |= (1 << WGM21); //Imposta il timer in modalità di confronto con OCR2A
  TCCR2B |= (1 << CS22); // Imposta il prescaler a 64
  TIMSK2 |= (1 << OCIE2A); // Abilita l'interrupt per il confronto con OCR2A

  // Configurazione dell'interrupt di overflow del timer 0
  TIMSK0 |= (1 << TOIE0); // Abilita l'interrupt di overflow del timer 0
  // Configurazione del timer 0
  TCCR0A |= (1 << WGM00) | (1 << WGM01); // Modalità Fast PWM
  TCCR0B |= (1 << CS02); // Prescaler di 256 (frequenza di clock di 16MHz / 256)
  OCR0A = 255; // Valore massimo del contatore (periodo del segnale PWM)

  sei(); // Abilita gli interrupt dopo la configurazione

  DDRD |= (1 << PWM_PIN);//Imposta il pin 5 che genera il segnale PWM come output

  TCCR0A = _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); //impostazione del PWM
  TCCR0B = _BV(CS01); //impostazione della frequenza di clock a 64

   // Abilita la tensione di riferimento AVCC (5V)
  ADMUX |= (1 << REFS0);
  // Imposta la prescaler su 128 per ottenere una frequenza di campionamento di 125 kHz
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  // Abilita il convertitore ADC
  ADCSRA |= (1 << ADEN);

  // Inizializzazione dei pin del display come output
  DDRB |= (1 << pinRS);
  DDRB |= (1 << pinEN);
  DDRB |= (1 << pinD4);
  DDRB |= (1 << pinD5);
  DDRB |= (1 << pinD6);
  DDRD |= (1 << pinD7);

  // Abilita il pushButton Movement come input-pullup
  DDRD &= ~(1 << movementPin);
  // Imposta il pull-up interno
  PORTD |= (1 << movementPin);

  //Abilita i pin del pushButton Start come input-pullup
  // Imposta il pin come input
  DDRD &= ~(1 << startStopButton);
  // Imposta il pull-up interno
  PORTD |= (1 << startStopButton);

  // Inizializzazione del display LCD
  sendCommand(0x33); // Sequenza di inizializzazione 8-bit
  sendCommand(0x32); // Sequenza di inizializzazione 4-bit
  sendCommand(0x28); // Modalità 2 linee, carattere 5x8
  // Abilita il display LCD
  sendCommand(0x0C);
  // Pulisce il display LCD
  sendCommand(0x01);
  // Imposta il contrasto iniziale del display
  sendCommand(0x0C); // Display acceso, cursore spento, lampeggio spento
  sendCommand(0x06); // Incremento del cursore
  sendCommand(0x01); // Pulizia del display

  // Posizionamento del cursore alla prima riga, primo carattere
  sendCommand(0x80);
  // Scrittura del messaggio
  sendText("Push the button");
  // Posizionamento del cursore alla seconda riga, primo carattere
  sendCommand(0xC4);
  sendText("to start");

}

void loop() {
    // Avvia la conversione ADC per il sensore LDR
  ADCSRA |= (1 << ADSC);
  // Attendi la fine della conversione ADC
  while (ADCSRA & (1 << ADSC));
  // Leggi il valore convertito dal sensore LDR
  ldrValue = ADC;

  OCR0B = map(ldrValue, 0, 1023, 255, 0); //conversione del valore letto in un valore PWM scrivendolo sul registro OCR0B

  if (startStopButtonPressed) {

    if (transitionDetected) {
      T = milliseconds - lastRevolutionStartTime; // computa il tempo per fare una rivoluzione
      distance = (nRevolution * bicycleWheelCircumference) / 1000; //calcola la distanza
      currentSpeed = (bicycleWheelCircumference / T) * 3600; //calcola la velocità
      transitionDetected = false; //reimposta il valore booleano a false
    }

    T1 = milliseconds - lastRevolutionStartTime; //per leggere il tempo trascorso dall'ultimo impulso 
    //anche se non è stata detectata una transizione
    updateLCD(); //aggiorna il display con le informazioni di tempo 
  }
  if (startStopButtonPressedLong) {
    // Esegui il reset dell'Arduino
    asm("jmp 0");
    startStopButtonPressedLong = false; //reimposta il valore a false
  }
}

//interrupt service routine per il sensore LDR 
ISR(INT1_vect) {
  if ((!(PIND & (1 << movementPin)))) { //button pressed
    transitionDetected = true;
    nRevolution++;
  } else //if (PIND & (1 << movementPin)) 
  {// button released
    lastRevolutionStartTime = milliseconds;
  }
}

//interrupt service routine per startStopButton
ISR(INT0_vect) {
  if (!(PIND & (1 << startStopButton))) { // Button pressed
    buttonPressStartTime = milliseconds;
  } else { // Button released

    unsigned long buttonPressDuration = milliseconds - buttonPressStartTime; //durata dellla pressione del pulsante

    if (buttonPressDuration >= 1000) { // se la pressione è >= 1 secondo rileva una lunga pressione
      startStopButtonPressedLong = true;
    } else if (buttonPressDuration > 0 && buttonPressDuration < 1000) { //se è minore di un secondo, ma è comunque stato premuto
      startStopButtonPressed = true;
    }
  }
}

//interrupt service routine for the counter
ISR(TIMER1_COMPA_vect) {
  if (startStopButtonPressed) {
    seconds++;
  } // fai partire il contatore di secondi solo se il programma è stato avviato
}

// Interrupt del timer2 per il conteggio dei millisecondi
ISR(TIMER2_COMPA_vect) {
  milliseconds++;
}

void sendCommand(byte command) {
  PORTB &= ~_BV(pinRS); // Impostazione del pin RS a basso per i comandi 
  sendData(command);
}

void sendNum(volatile float num) {
  char buffer[10];
  dtostrf(num, 1, 0, buffer); // Converte il numero in formato stringa con 5 cifre totali e 2 decimali
  sendText(buffer);
}

void sendNumFloat(volatile float num) {
  char buffer[10];
  dtostrf(num, 5, 2, buffer); // Converte il numero in formato stringa con 1 cifra totale
  sendText(buffer);
}

void sendText(const char * text) {
  PORTB |= _BV(pinRS); // Impostazione del pin RS ad alto per i dati
  while ( * text) {
    sendData( * text);
    text++;
  }
}

void sendData(byte data) {
  PORTB &=~_BV(3); //EN pin su LOW -> Accensione del display
  
// Invio dei 4 bit piu significativi, primo nibble
// Imposta il pin D4 con il bit meno significativo di 'data'
  PORTB = (PORTB & ~_BV(pinD4)) | (((data >> 4) & 1) << 2);
// Imposta il pin D5 con il secondo bit meno significativo di 'data'
  PORTB = (PORTB & ~_BV(pinD5)) | (((data >> 5) & 1) << 1);
// Imposta il pin D6 con il terzo bit meno significativo di 'data'
  PORTB = (PORTB & ~_BV(pinD6)) | (((data >> 6) & 1) << 0);
// Imposta il pin D7 con il quarto bit meno significativo di 'data'
  PORTD = (PORTD & ~_BV(pinD7)) | (((data >> 7) & 1) << 7);
  
  PORTB |= _BV(pinEN); //EN pin su HIGH -> Spegnimento del display
  PORTB &=~_BV(pinEN); //EN pin su LOW -> Accensione del display
  
  // Invio dei 4 bit meno significativi, secondo nibble
// Imposta il pin D4 con il bit meno significativo di 'data'
  PORTB = (PORTB & ~_BV(pinD4)) | (((data >> 0) & 1) << 2);
// Imposta il pin D5 con il secondo bit meno significativo di 'data'
  PORTB = (PORTB & ~_BV(pinD5)) | (((data >> 1) & 1) << 1);
// Imposta il pin D6 con il terzo bit meno significativo di 'data'
  PORTB = (PORTB & ~_BV(pinD6)) | (((data >> 2) & 1) << 0);
// Imposta il pin D7 con il quarto bit meno significativo di 'data'
  PORTD = (PORTD & ~_BV(pinD7)) | (((data >> 3) & 1) << 7);
  
  PORTB |= _BV(3); //EN pin su HIGH -> Spegnimento del display
}

void updateLCD() {
  // Posizionamento del cursore alla prima riga, primo carattere
  sendCommand(0x80);
  // Scrittura del messaggio
  sendText("Speed:    ");

  if (T1 > 0 && T1 < 10000) {
    sendNum(currentSpeed);
  } else //if (T1 > 10000) 
  {
    sendNum(0);
  } //se il tempo trascorso dall'ultimo immpulso è >=10 secondi allora 
  //significa che l'utente è fermo e riporta la velocità istantanea a 0

  sendText("km/h");

  // Calcola le ore, i minuti e i secondi
  int hours = seconds / 3600;
  int minutes = (seconds / 60) % 60;
  int second = seconds % 60;

  // Posizionamento del cursore alla seconda riga, primo carattere
  sendCommand(0xC0);
  //sendText("Time: ");
  sendNum(hours);
  sendText(":");
  if (minutes < 10) {
    sendNum(0); // Aggiunge uno zero iniziale se i minuti sono inferiori a 10
  }
  sendNum(minutes);
  sendText(":");
  if (second < 10) {
    sendNum(0); // Aggiunge uno zero iniziale se i secondi sono inferiori a 10
  }
  sendNum(second);

  sendText("  "); //aggiungi uno spazio

  sendNumFloat(distance);
  sendText("km");
}