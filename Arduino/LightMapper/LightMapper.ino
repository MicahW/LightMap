#include <SoftwareSerial.h>
SoftwareSerial BTserial(2, 3); // RX | TX
 
void setup() 
{
#ifdef DEBUG
Serial.begin(9600);
#endif
    BTserial.begin(9600);  // Init bluetooth serial connection
}

// Read a length encoded serial message over bluetooth
// buf: message buffer to read into
// return: num bytes read
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

// Read a length encoded serial message from bluetooth
// buf: message buffer to read from
// size: message size
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
