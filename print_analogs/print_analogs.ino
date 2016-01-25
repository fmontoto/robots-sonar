#define trigPin 8
#define echoPin 2 // Only 2 or 3, as we're using interruptions

#define analogPin A3
#define SerialBAUD 9600
#define VALUES_TO_STORE 200

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

void send_standard_distance(int dist) {
  Serial.println(dist);
}

int* new_method_distance() {
  static int values[VALUES_TO_STORE];
  keep_looping = 1;
  int n = 1;
  trigger_sonar();
  while(digitalRead(echoPin) == HIGH)
    ;
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
  values[0] = n;
  return &values[0];
}

void send_new_method_distance(int *values) {
  int i;
  int total_values = values[0];
  for(i = 1; i < total_values; i++) {
    Serial.println(values[i]);
    delay(50);
  }
  Serial.println(-2);
}

void loop() {
  char input[2];
  int *aux;
  while(Serial.available() < 2) {
    delay(500);
  }

  Serial.readBytes(input, 2);
  if(strncmp(input, "ND", 2) == 0) {
    Serial.print("OK");
    aux = new_method_distance();
    send_new_method_distance(aux);
  }
  else if(strncmp(input, "SD", 2) == 0) {
    Serial.print("OK");
    int dist = standard_distance();
    send_standard_distance(dist);
  }
  else if(strncmp(input, "AL", 2) == 0) {
    Serial.print("YES");
  }
  else {
    Serial.print("Not a valid input");
  }
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
