// #include <QueueArray.h>

// #include <SoftwareSerial.h>
#include <Servo.h>
#include <DallasTemperature.h>
#define LED LED_BUILTIN
#define LED2 11
#define LED3 6
#define HEAT 7
#define wifi Serial
#define SERVO 10
#define BUFFER_SIZE 64
#define TEMPERATURE_BUFFER_SIZE 256
#define i32 int32_t
#define TIMEOUT 7000 // mS
#define MessageQueueSize 8 // Must be power of 2

struct Finder {
  const char* value;
  byte index;
  byte length;

  Finder (const char* needle) {
    value = needle;
    index = 0;
    length = strlen(needle);
  }

  bool find() {
    while(Serial.available()) {
      if (find(Serial.read())) return true;
    }
    return false;
  }
  
  bool find(char c) {
    if (value[index] == c) {
      // Eat
      index++;

      if (index == length) {
        index == 0;
        return true;
      }
    } else if (index == 0) {
      // Eat
    } else {
      // Assume a "nice" needle string. This will not work for every possible string
      index = 0;
      return find(c);
    }
    return false;
  }
};

enum Stage : i32 {
  AlwaysOff = -1,
  HeatToHigh = 0,
  CoolToLow = 1,
  KeepAtFinal = 2,
  AlwaysOn = 3,
};

struct State {
  float lowTemp;
  float highTemp;
  float finalTemp;
  Stage stage;
};

// DS18B20 Temperature chip i/o
OneWire ds(9);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&ds);
Servo servo;

Finder packetStart("+IPD,");

State state = {
  0.0f,
  0.0f,
  0.0f,
  Stage::AlwaysOff
};

enum MessageType {
  InvalidMessage,
  SetState,
  Fasten,
  Unfasten,
  EnableLED,
  DisableLED,
  GetTemperatures
};

struct Message {
  State state;
  MessageType type;
  char channel;
};

void assert (bool shouldBeTrue) {
  if (!shouldBeTrue) {
    int end = millis() + 5000;
    while(millis() < end) {
      pulse(LED3, millis() / 100.0f);
      pulse(LED2, millis() / 100.0f);
      pulse(LED, millis() / 100.0f);
    }
  }
}

struct MessageQueue {
  Message buffer[MessageQueueSize];
  int start = 0;
  int cnt = 0;

  int count() {
    return cnt;
  }

  bool isEmpty () {
    return cnt == 0;
  }
  
  bool isFull () {
    return cnt == MessageQueueSize;
  }
  
  void enqueue(Message& message) {
    assert(!isFull());
    buffer[(start + cnt) & (MessageQueueSize - 1)] = message;
    cnt++;
  }

  Message dequeue() {
    assert(cnt > 0);
    int prevIndex = start;
    start = (start + 1) & (MessageQueueSize - 1);
    cnt--;
    return buffer[prevIndex];
  }
};

MessageQueue messageQueue;
bool initComplete = false;

static int maxT = 80;
static int minT = 20;

int messageDepth = 0;

char compressTemperature(float t) {
  int tmap = (int)((t - minT)*(255.0f/(maxT-minT)));
  return (char)max(min(tmap, 255), 0);
}

float decompressTemperature(char t) {
  return t * ((maxT - minT) / 255.0f) + minT;
}

char temperatureBuffer[TEMPERATURE_BUFFER_SIZE];
i32 temperatureIndex = 0;
float lastTemperature = 0;
unsigned long lastTemperatureTick = 0;
unsigned long lastTemperaturePollTick = 0;
bool conversionIsComplete = false;
DeviceAddress temperatureProbeAddress;
float speed = 0;
float targetSpeed = 1;
unsigned long changedSpeedStartTime = 0;
#define NORMAL_SPEED 1
#define SPEED_MULTIPLIER 100
#define SPEED_ZERO 1550

bool tickHeat () {
  switch(state.stage) {
    case HeatToHigh:
      if (lastTemperature > state.highTemp) state.stage = CoolToLow;
      return true;
    case CoolToLow:
      if (lastTemperature < state.lowTemp) state.stage = KeepAtFinal;
      return false;
    case KeepAtFinal: {
      bool heat = lastTemperature < state.finalTemp;
      
      // Only keep the lamp on a fraction of the time when close to the target destination
      // to avoid overshoot
      float fraction = (state.finalTemp - lastTemperature) / 3f;
      fraction = min(max(fraction, 0), 1);
  
      int period = 30 * 1000;
      heat &= fraction > (millis() % period) / (float)period;
      return heat;
    }
    case AlwaysOn:
      return true;
    case AlwaysOff:
    default:
      return false;
  }
}

unsigned long lastSpeedTick = 0;
void tickSpeed () {
  unsigned long dt = min(millis() - lastSpeedTick, 100);
  if (dt > 10) {
    lastSpeedTick = millis();
  
    if (millis() - changedSpeedStartTime > 8000) {
      targetSpeed = NORMAL_SPEED;
    }
  
    //targetSpeed += (NORMAL_SPEED - targetSpeed)*0.0004*dt;
    //speed += (targetSpeed - speed)*0.002*dt;
    float delta = min(abs(targetSpeed - speed), 4 * 0.001 * dt);
    float ds = (targetSpeed > speed ? delta : -delta);
    speed += ds;
    servo.write((speed * SPEED_MULTIPLIER) + SPEED_ZERO);
  }
}

void testForSanity() {
  if (abs(decompressTemperature(compressTemperature(30)) - 30) > 0.26f) {
    Serial.println(F("Unit test failed"));
    flash(100);
  }
}

void configureSensors () {
  sensors.begin();
  sensors.setWaitForConversion(false);
  if (!sensors.getAddress(temperatureProbeAddress, 0)) {
    Serial.println("Could not find temperature sensor!");
    flash(100);
  }
  sensors.requestTemperatures();

  servo.attach(SERVO);
  tickSpeed();
}

void configurePins () {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);

  pinMode(HEAT, OUTPUT);
  digitalWrite(HEAT, HIGH);
  delay(100);
  digitalWrite(HEAT, LOW);
}

void configureWifi() {
  while(true) {
    digitalWrite(LED3, HIGH);
    delay(50);
    digitalWrite(LED3, LOW);
    
    if (!sendCommand("AT+RST", "ready", 7000)) continue;
    delay(1000);
    if (!sendCommand("AT", "OK", 7000)) continue;
    if (!sendCommand("AT+CWMODE=1","OK")) continue;
    if (!sendCommand("AT+CWJAP=\"icosikaitetragon\",\"qwertydvorak\"", "OK", 7000)) continue;
    // sendCommand("AT+CIFSR", "OK");
    if (!sendCommand("AT+CIPMUX=1","OK")) continue;
    if (!sendCommand("AT+CIPSERVER=1,80","OK")) continue;

    digitalWrite(LED3, HIGH);
    break;
  }
}

void setup() {
  wifi.begin(115200);
  Serial.setTimeout(TIMEOUT);

  testForSanity();
  configureSensors();
  configurePins();
  configureWifi();

  initComplete = true;
}

char buffer[BUFFER_SIZE];

void flash(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED, HIGH);
    delay(20);
    digitalWrite(LED, LOW);
    delay(50);
  }
}

int failedPolls = 0;

void pollTemperature() {
  if (failedPolls >= 5) {
    failedPolls = 0;
    conversionIsComplete = true;
    sensors.requestTemperatures();
  }

  if (!conversionIsComplete) {
    conversionIsComplete = sensors.isConversionComplete();
  }

  if (conversionIsComplete) {
    // Bus is very unreliable so we may have to retry several times
    float temp = sensors.getTempC(temperatureProbeAddress);
    
    if(temp == DEVICE_DISCONNECTED_C) {
      // Received invalid data from temperature probe. Trying again
      failedPolls++;
      //Serial.println(F("Received invalid data from temperature probe. Trying again"));
    } else if (temp < 0) {
      failedPolls++;
      Serial.print(F("Found invalid temperature: "));
      Serial.println(temp);
    } else {
      failedPolls = 0;
      lastTemperature = temp;
      // Serial.println("Temp: " + String(temp, DEC));
      conversionIsComplete = false;
      sensors.requestTemperatures();
    }
  }
}

void loop() {
  unsigned long ms = millis();
  
  if (ms - lastTemperaturePollTick > 200) {
    pollTemperature();
    lastTemperaturePollTick = ms;
  }

  if (ms - lastTemperatureTick > 5000) {
    lastTemperatureTick = ms;
    if (temperatureIndex >= TEMPERATURE_BUFFER_SIZE) {
      // Shift everything by 1 step
      for (int i = 1; i < TEMPERATURE_BUFFER_SIZE; i++) {
        temperatureBuffer[i-1] = temperatureBuffer[i];
      }
    }
    temperatureIndex++;
  }

  // Update the temperature index in the array
  if (temperatureIndex >= TEMPERATURE_BUFFER_SIZE) {
    temperatureBuffer[TEMPERATURE_BUFFER_SIZE-1] = compressTemperature(lastTemperature);
  } else {
    temperatureBuffer[temperatureIndex] = compressTemperature(lastTemperature);
  }

  tickSpeed();
  pollNetwork(true, 'X');

  digitalWrite(HEAT, tickHeat() ? HIGH : LOW);

  if(messageDepth == 0 && !messageQueue.isEmpty()) {
    handleMessage(messageQueue.dequeue());
  }
}

void pollNetwork (bool useGlobalWifiInput, char wifiInput) {
  pulse(LED3, millis() / 500.0f);
  if (useGlobalWifiInput ? packetStart.find() : packetStart.find(wifiInput)) {
    handlePacket();
  }
}

int timedRead () {
  while(!wifi.available());
  return wifi.read();
}

void handlePacket () {
  char channel = (char)timedRead();
  timedRead(); // Discard comma ','

  int len = wifi.parseInt();
  char comma = (char)timedRead();
  assert(':' == comma);

  if (len + 1 >= BUFFER_SIZE) {
    // Discard. Message is too large
    for(int i = 0; i < len; i++) wifi.read();
    response("Message too long", channel);
    close(channel);
    return;
  }

  // Serial.println("Rec " + String(len,DEC));
  
  int read = 0;
  while (read < len) {
    read += wifi.readBytes(buffer + read, len - read);
    // wifi.println("Just read: " + String(read, DEC) + " bytes out of " + String(len, DEC));
  }

  if (messageQueue.count() >= MessageQueueSize) {
    response("Too many messages too quickly " + String(messageDepth, DEC), channel);
    close(channel);
    return;
  }

  Message message;
  message.channel = channel;

  if (len == strlen("S=") + sizeof(State) && buffer[0] == 'S' && buffer[1] == '=') {
    message.type = SetState;
    memcpy(&message.state, buffer + strlen("S="), sizeof(State));
    messageQueue.enqueue(message);
    return;
  }
  
  // Ensure the message did not contain any null bytes
  for(int i = 0; i < len; i++) {
    if(buffer[i] == '\0') {
      buffer[i] = '_';
    }
  }

  // Null-terminate the string
  buffer[len] = '\0';

  if (startsWith(buffer, "GET TEMPERATURES")) {
    message.type = GetTemperatures;
  } else if (startsWith(buffer, "fasten")) {
    message.type = Fasten;
  } else if (startsWith(buffer, "unfasten")) {
    message.type = Unfasten;
  } else if (startsWith(buffer, "LED=ON")) {
    message.type = EnableLED;
  } else if (startsWith(buffer, "LED=OFF")) {
    message.type = DisableLED;
  } else {
    response("Unknown message", channel);
    close(channel);
    return;
  }
  
  messageQueue.enqueue(message);
  //Serial.print(F("Message queue size: "));
  //Serial.println(messageQueue.count());
  //servo.write(incomingString.toInt());
}

bool startsWith(const char* str, const char* needle) {
  return strncmp(str, needle, strlen(needle)) == 0;
}

void writei32toWifi(i32 v) {
  wifi.write((byte*)&v, 4);
}

void handleMessage (struct Message message) {
  char channel = message.channel;
  switch(message.type) {
    case SetState: {
      state = message.state;
      response("OK", channel);
      close(channel);
      break;
    }
    case GetTemperatures: {
      int temperatureValues = temperatureIndex + 1;
      int bytesToSend = min(TEMPERATURE_BUFFER_SIZE, temperatureValues);
      wifi.print("AT+CIPSEND=");
      wifi.write(channel);
      wifi.write(',');
      wifi.println(bytesToSend + 4,DEC);
      listenForACK("Send T", "OK", 500);
      writei32toWifi(temperatureValues - bytesToSend);
      wifi.write(temperatureBuffer, bytesToSend);
      listenForACK("Send T+", "SEND OK", 1000);
      close(channel);
      break;
    }
    case Fasten: {
      response("OK", channel);
      close(channel);
      targetSpeed = 4;
      changedSpeedStartTime = millis();
      break;
    }
    case Unfasten: {
      response("OK", channel);
      close(channel);
      targetSpeed = -2;
      changedSpeedStartTime = millis();
      break;
    }
    case EnableLED: {
      digitalWrite(LED, HIGH);
      response("OK", channel);
      close(channel);
      break;
    }
    case DisableLED: {
      digitalWrite(LED, LOW);
      response("OK", channel);
      close(channel);
      break;
    }
  }
}

void responsiveDelay(int ms) {
  if (initComplete) {
    wifi.println("Ticking...");
    messageDepth++;
    unsigned long end = millis() + ms;
    while(millis() < end) {
      digitalWrite(LED2, HIGH);
      loop();
      digitalWrite(LED2, LOW);
    }
    messageDepth--;
  } else {
    delay(ms);
  }
}

void close(char channel) {
  wifi.print(F("AT+CIPCLOSE="));
  wifi.write(channel);
  wifi.println();
  listenForACK("Close", "OK");
}

void response(const char* data, char channel) {
  //sendCommand("AT+CIPSEND=" + String(channel) + "," + String(data.length(),DEC), "OK");
  //sendCommand(data, "SEND OK");
  
  wifi.print(F("AT+CIPSEND="));
  wifi.write(channel);
  wifi.write(',');
  wifi.println(strlen(data));
  listenForACK("Send...", "OK");
  sendCommand(data, "SEND OK");
}

void response(String data, char channel) {
  //sendCommand("AT+CIPSEND=" + String(channel) + "," + String(data.length(),DEC), "OK");
  //sendCommand(data, "SEND OK");
  
  wifi.print(F("AT+CIPSEND="));
  wifi.write(channel);
  wifi.write(',');
  wifi.println(data.length());
  listenForACK("Send...", "OK");
  sendCommand(data, "SEND OK");
}

boolean sendCommand(const char* cmd, const char* ack){
  wifi.println(cmd);
  return listenForACK(cmd, ack);
}

boolean sendCommand(String& cmd, const char* ack){
  wifi.println(cmd);
  return listenForACK(cmd, ack, 5000);
}

boolean sendCommand(const char* cmd, const char* ack, int timeout){
  wifi.println(cmd); // Send "AT+" command to module
  return listenForACK(cmd, ack, timeout);
}

boolean listenForACK(const char* cmd, const char* ack) {
  return listenForACK(cmd, ack, 5000);
}

boolean listenForACK(const char* cmd, const char* ack, int timeout) {
  if(!echoFind(ack, timeout)) {
    Serial.print(F("Command '"));
    Serial.print(cmd);
    Serial.println(F("' timed out"));

    int startMillis = millis();
    while(millis() - startMillis < 3500) {
      pulse(LED2, (millis() - startMillis) / 100.0);
    }
    digitalWrite(LED2, LOW);
    return false;
  }
  return true;
}


boolean listenForACK(String& cmd, const char* ack, int timeout) {
  if(!echoFind(ack, timeout)) {
    Serial.print(F("Command '"));
    Serial.print(cmd);
    Serial.println(F("' timed out"));

    int startMillis = millis();
    while(millis() - startMillis < 3500) {
      pulse(LED2, (millis() - startMillis) / 100.0);
    }
    digitalWrite(LED2, LOW);
    return false;
  }
  return true;
}

void pulse(int pin, float t) {
  float alpha = sin(t*0.5);
  alpha *= alpha;
  analogWrite(pin, (int)(alpha*255));
}

boolean echoFind(const char* keyword, int timeout){
  Finder finder(keyword);
  unsigned long startMillis = millis();
  unsigned long lastMillis = lastMillis;
  int cnt = 0;
  int ledDelay = 400;
  while(true) {
    while(wifi.available()) {
      int data = wifi.read();
      if(finder.find(data)) {
        digitalWrite(LED2, LOW);
        return true;
      }

      if (initComplete) {
        messageDepth++;
        pollNetwork(false, data);
        messageDepth--;
      }
    }

    cnt = (cnt + 1) % 255;
    unsigned long elapsed = millis() - startMillis;
    if (elapsed > ledDelay) {
      pulse(LED2, (elapsed - ledDelay) / 500.0);
    }

    if (elapsed > timeout) {
      digitalWrite(LED2, LOW);
      return false;
    }
  }
}
