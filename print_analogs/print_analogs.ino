#define trigPin 8
#define echoPin 2 // Only 2 or 3, as we're using interruptions

#define analogPin A3
#define SerialBAUD 230400
#define VALUES_TO_STORE 300

char buffer[100];
volatile int keep_looping;

void stop_looping() {
  keep_looping = 0;
}

void setup() {
  Serial.begin(SerialBAUD);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void trigger_sonar() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}

/** Calculates the distance using the sonar.
* @return distance en centimeters, 0 if the distance ids out of range.
*/
int standard_distance() {
  int distance, duration;
  trigger_sonar();
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  if (distance >= 200 || distance <= 0){
    return 0;
  }
  return distance; 
}

void new_method_distance() {
  static int values[VALUES_TO_STORE];
  keep_looping = 1;
  int n = 0;
  trigger_sonar();
  while(digitalRead(echoPin) == HIGH)
    Serial.println(analogRead(analogPin));;
  attachInterrupt(digitalPinToInterrupt(echoPin), stop_looping, FALLING);
  while(keep_looping && n < VALUES_TO_STORE) {
    values[n] = analogRead(analogPin);
    ++n;
  }
  values[n++] = -1;
  while(n < VALUES_TO_STORE) {
    values[n] = analogRead(analogPin);
    ++n;
  }
  detachInterrupt(digitalPinToInterrupt(echoPin));
  return values
}

void loop() {
  //Serial.println(standard_distance());
  new_method_distance();
  delay(100);
}

void loop_() {
  int analog_val;
  unsigned long start_time, current_time, elapsed_time;
  long duration, distance;
  int available_bytes;
  int i;
  int n = 100;

  trigger_sonar();
  
  while(digitalRead(echoPin) == LOW) {
    ;
  }
  // Aqui comienza la medicion
  start_time = micros();
  n = 0;
  while(digitalRead(echoPin) == HIGH) {
    Serial.println(analogRead(A3));
    Serial.println(n++);
  }
  Serial.println("Referencia termino");
  n = 50;
  while(--n) {
    Serial.println(analogRead(A3));
  }
  
  current_time = micros();
  elapsed_time = current_time - start_time;
  duration = elapsed_time;
  Serial.print(duration);
  Serial.println("asd");
  //duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  if (distance >= 200 || distance <= 0){
    Serial.println("Out of range");
  }
  else {
    Serial.print(distance);
    Serial.println(" cm");
  }
  
  delay(100);
}
