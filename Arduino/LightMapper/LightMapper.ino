#include <SoftwareSerial.h>
SoftwareSerial BTserial(2, 3); // RX | TX
 
void setup() 
{
    BTserial.begin(9600);  // Init bluetooth serial connection
}

// If a message is avaiable then read it and return num bytes read
// else return 0
int readBT(char* buf) {
  if (!BTserial.available()) {
    return 0;
  }
  char message_size = BTserial.read();
  for (char pos = 0; pos < message_size; pos++) {
    buf[pos] = BTserial.read();    
  }
  buf[message_size] = '\0';
  return message_size;
}

// Write the message
void writeBT(char* buf, int size) {
  BTserial.write((char) size);
  for (int i = 0; i < size; i++) {
    BTserial.write(buf[i]);
  }
}
 
void loop()
{
  char message_buf[256];
  readBT(message_buf);
}
