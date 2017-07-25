// #include <SoftwareSerial.h>
#define LED LED_BUILTIN
#define wifi Serial
#define BUFFER_SIZE 256

//SoftwareSerial wifi(7, 6); // RX, TX
#define TIMEOUT 7000 // mS

//String buffer = "";

void print(String msg) {
  //buffer +=  "MSG: " + msg + ";";
  //out.print(msg);
}

void println() {
  //buffer += "\n";
  //out.println();
  flush();
}

void println(String msg) {
  //buffer += "MSG: " + msg + ";\n";
  //out.print(msg);
  flush();
}

void flush() {
  //Serial.print(buffer);
  //buffer = "";
}

void setup() {
  Serial.setTimeout(TIMEOUT);
  
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  pinMode(7, INPUT);
  pinMode(6, OUTPUT);
  
  //Serial.begin(115200);
  wifi.begin(115200);
  sendCommand("AT+RST", "ready");
  delay(3000);
  sendCommand("AT", "OK");
  sendCommand("AT+CWMODE=1","OK");
  sendCommand("AT+CWJAP=\"icosikaitetragon\",\"qwertydvorak\"", "OK");
  // sendCommand("AT+CIFSR", "OK");
  sendCommand("AT+CIPMUX=1","OK");
  sendCommand("AT+CIPSERVER=1,80","OK");
  flash(10);
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

void loop() {
  // put your main code here, to run repeatedly:
  String incomingString = "";

  if (!echoFind("+IPD,0,")) {
    flash(2);
    return;
  }

  String tmp = wifi.readStringUntil(':');
  int len = tmp.toInt();

  if (len + 1 >= BUFFER_SIZE) {
    // Discard. Message is too large
    for(int i = 0; i < len; i++) wifi.read();
    return;
  }
  
  int read = 0;
  while (read < len) {
    read += wifi.readBytes(buffer + read, len - read);
    // wifi.println("Just read: " + String(read, DEC) + " bytes out of " + String(len, DEC));
  }

  // Ensure the message did not contain any null bytes
  for(int i = 0; i < len; i++) {
    if(buffer[i] == '\0') {
      buffer[i] = '_';
    }
  }

  // Null-terminate the string
  buffer[len] = '\0';
  
  //println("Received String: " + incomingString);
  //println("Hash: " + String(hash,DEC));
  //flush();

  incomingString = String(buffer);
  if(len != incomingString.length()) {
    response("INTERNAL SERVER ERROR " + String(len,DEC) + " != " + String(incomingString.length(),DEC));
    return;
  }

  // String output = String("Received string: ") + buffer + " (length: " + String(len, DEC) + ")\n";
  response("OK\n");
  if (incomingString.indexOf("LED=ON") != -1) {
    digitalWrite(LED, HIGH);
  }

  if (incomingString.indexOf("LED=OFF") != -1) {
    digitalWrite(LED, LOW);
  }
}

void response(String data) {
  sendCommand("AT+CIPSEND=0," + String(data.length(),DEC), "OK");
  sendCommand(data, "SEND OK");
}

boolean sendCommand(String cmd, const char* ack){
  // println("Sending: " + cmd);
  wifi.println(cmd); // Send "AT+" command to module
  if(!echoFind(ack)) {
    println("Command '" + cmd + "' timed out");

    for(int i = 0; i < 100; i++) {
      digitalWrite(LED, LOW);
      delay(50);
      digitalWrite(LED, HIGH);
      delay(50);
    }
    return false;
  }
  return true;
}

boolean echoFind(const char* keyword){
  return Serial.find((char*)keyword);
  /*
  byte current_char = 0;
  byte keyword_length = keyword.length();
  long deadline = millis() + TIMEOUT;
  String buff = "";
  while(millis() < deadline){
    if (wifi.available()){
      char ch = wifi.read();

      //buff += ch;
      if (ch == keyword[current_char]) {
        if (++current_char == keyword_length){
          println("Read: '" + buff + "'");
          println("-> Found ACK");
          return true;
        }
      }
    }
  }
  println("Read: " + buff);
  println();
  return false; // Timed out*/
}
