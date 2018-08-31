#include <ESP8266_AT_Client.h>
#include <Print.h>
#include <string.h>

// #define ESP8266_AT_CLIENT_ENABLE_DEBUG
// #define ESP8266_AT_CLIENT_DEBUG_ECHO_EVERYTHING
// #define ESP8266_AT_CLIENT_DEBUG_OUTGOING
// #define ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES
// #define ESP8266_AT_CLIENT_DEBUG_INCOMING

static uint16_t ESP8266_AT_Client::bytesAvailableMax = 0;
#if defined(ESP8266_AT_CLIENT_DEBUG_INCOMING)
#define DEBUG_WINDOW_SIZE (32)

static uint8_t debug_read_window_data[DEBUG_WINDOW_SIZE] = {};
uint8_t* ESP8266_AT_Client::debug_read_window = debug_read_window_data;
static uint16_t ESP8266_AT_Client::debug_read_window_index = 0;

static uint8_t debug_write_window_data[DEBUG_WINDOW_SIZE] = {};
uint8_t* ESP8266_AT_Client::debug_write_window = debug_write_window_data;
static uint16_t ESP8266_AT_Client::debug_write_window_index = 0;

static void ESP8266_AT_Client::printDebugWindow() {  
  Serial.println("::BEGIN HISTORY::");
  Serial.print("MAX STREAM AVAILABLE: ");
  Serial.println(bytesAvailableMax);
  Serial.println("::READ HISTORY::");
  uint16_t idx = debug_read_window_index;
  uint16_t count = 0;
  while(count < DEBUG_WINDOW_SIZE){
    uint8_t b = debug_read_window[idx] & 0xff;
    if(isprint(b) || isspace(b)){
      Serial.print((char) b);
    }
    else{
      Serial.print(" 0x");
      if(b < 0x10) Serial.print('0');
      Serial.print(b, HEX);
    }

    idx++;
    if(idx >= DEBUG_WINDOW_SIZE){
      idx = 0;
    }

    count++;
  }

  delay(1000);
  Serial.println("\n::UNREAD HISTORY::");
  while(Serial1.available()){
    uint8_t b = Serial1.read() & 0xff;
    if(isprint(b) || isspace(b)){
      Serial.print((char) b);
    }
    else{
      Serial.print(" 0x");
      if(b < 0x10) Serial.print('0');
      Serial.print(b, HEX);
    }
  }

  Serial.println("\n::WRITE HISTORY::");
  idx = debug_write_window_index;
  count = 0;
  while(count < DEBUG_WINDOW_SIZE){
    uint8_t b = debug_write_window[idx] & 0xff;
    if(isprint(b) || isspace(b)){
      Serial.print((char) b);
    }
    else{
      Serial.print(" 0x");
      if(b < 0x10) Serial.print('0');
      Serial.print(b, HEX);
    }

    idx++;
    if(idx >= DEBUG_WINDOW_SIZE){
      idx = 0;
    }

    count++;
  }

  Serial.println("\n::END HISTORY::");
  for(;;){
    // service the watchdog
    digitalWrite(14, LOW);
    delay(3);    
    digitalWrite(14, HIGH);      
    delay(500);
  } // die
}

static void ESP8266_AT_Client::addToDebugReadWindow(uint8_t b){
  debug_read_window[debug_read_window_index++] = b;
  if(debug_read_window_index >= DEBUG_WINDOW_SIZE){
    debug_read_window_index = 0;
  }
}

static void ESP8266_AT_Client::addToDebugWriteWindow(uint8_t b){
  debug_write_window[debug_write_window_index++] = b;
  if(debug_write_window_index >= DEBUG_WINDOW_SIZE){
    debug_write_window_index = 0;
  }
}

#else
static void ESP8266_AT_Client::printDebugWindow() { }
static void ESP8266_AT_Client::addToDebugReadWindow(uint8_t b){ }
static void ESP8266_AT_Client::addToDebugWriteWindow(uint8_t b){ }
#endif

ESP8266_AT_Client::ESP8266_AT_Client(uint8_t enable_pin){
  this->stream = &Serial; // default assumption
  this->streamAsPrint = (Print *) this->stream;
  this->socket_connected = false;
  this->wifi_is_connected = false;
  this->ok_flag = false;
  this->error_flag = false;
  this->ready_flag = false;
  this->send_ok_flag = false;
  this->listener_started = false;
  this->input_buffer = NULL;
  this->input_buffer_length = 0;
  this->input_buffer_read_idx = 0;
  this->input_buffer_write_idx = 0;
  this->enable_pin = enable_pin;
  this->num_consumed_bytes_in_input_buffer = 0;
  this->num_free_bytes_in_input_buffer = 0;
  this->debugEnabled = false;
  this->debugStream = NULL;
  this->tcp_keep_alive_interval_seconds = 120;
}

ESP8266_AT_Client::ESP8266_AT_Client(uint8_t enable_pin, Stream * stream){
  this->stream = stream;
  this->streamAsPrint = (Print *) this->stream;
  this->socket_connected = false;
  this->wifi_is_connected = false;
  this->ok_flag = false;
  this->error_flag = false;  
  this->ready_flag = false;
  this->send_ok_flag = false;
  this->listener_started = false;
  this->input_buffer = NULL;
  this->input_buffer_length = 0;
  this->input_buffer_read_idx = 0;
  this->input_buffer_write_idx = 0;
  this->enable_pin = enable_pin;
  this->num_consumed_bytes_in_input_buffer = 0;
  this->num_free_bytes_in_input_buffer = 0;
  this->debugEnabled = false;
  this->debugStream = NULL;
  this->tcp_keep_alive_interval_seconds = 120;
}

ESP8266_AT_Client::ESP8266_AT_Client(uint8_t enable_pin, Stream * stream, uint8_t * buf, uint16_t buf_length){
  this->stream = stream;
  this->streamAsPrint = (Print *) this->stream;
  this->socket_connected = false;
  this->wifi_is_connected = false;
  this->ok_flag = false;
  this->error_flag = false;
  this->ready_flag = false;
  this->send_ok_flag = false;
  this->listener_started = false;
  this->input_buffer = buf;
  this->input_buffer_length = buf_length;
  this->input_buffer_read_idx = 0;
  this->input_buffer_write_idx = 0;
  this->enable_pin = enable_pin;
  this->num_consumed_bytes_in_input_buffer = 0;
  this->num_free_bytes_in_input_buffer = 0;
  this->debugEnabled = false;
  this->debugStream = NULL;
  this->tcp_keep_alive_interval_seconds = 120;
}

void ESP8266_AT_Client::setDebugStream(Stream * ds){
  debugStream = ds;
}

boolean ESP8266_AT_Client::reset(void){
  // Serial.print("Available For Write");
  // Serial.println(this->streamAsPrint->availableForWrite());  

  // reset the buffer state pointers
  input_buffer_read_idx = 0;
  input_buffer_write_idx = 0;  
  num_consumed_bytes_in_input_buffer = 0;
  num_free_bytes_in_input_buffer = input_buffer_length;
  numIncomingBytesPending = 0;

  socket_connected = false;
  wifi_is_connected = false;
  listener_started = false;
  ok_flag = false;
  error_flag = false;
  ready_flag = false;  
  send_ok_flag = false;

  pinMode(enable_pin, OUTPUT);
  digitalWrite(enable_pin, LOW);
  delay(50);
  digitalWrite(enable_pin, HIGH);

  ESP8266_DEBUG("ESP8266 Hello World.");

  // wait for ok or error or timeout
  // because 'ready' triggers the ok_flag
  const int32_t interval = 10000;
  uint32_t current_millis = millis();
  uint32_t previous_millis = current_millis;
  boolean timeout_flag = false;
  while(!error_flag && !ready_flag && !timeout_flag){
    current_millis = millis();

    if(stream->available() > 0){
      int16_t b = streamReadChar();
      if(b > 0){
        previous_millis = current_millis;      
      }       
    }   

    if (current_millis - previous_millis >= interval) {
      timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
      Serial.println("PANIC6");
      printDebugWindow();
#endif
    }  
  }  

  return ready_flag;
}

/** Set the stream where AT commands are sent and responses received
	@param stream     the AT stream (e.g. &Serial1)
	@return returns void
 */
void ESP8266_AT_Client::setStream(Stream * stream){
  this->stream = stream;
  this->streamAsPrint = (Print *) this->stream;
}

void ESP8266_AT_Client::setInputBuffer(uint8_t * buf, uint16_t buf_length){
  input_buffer = buf;
  input_buffer_length = buf_length;
  num_consumed_bytes_in_input_buffer = 0;
  num_free_bytes_in_input_buffer = buf_length;
  input_buffer_read_idx = 0;
  input_buffer_write_idx = 0;
}

int ESP8266_AT_Client::connect(IPAddress ip){
  return connect(ip, 80);
}

int ESP8266_AT_Client::connect(const char *host){
  return connect(host, 80);
}

/** Connect to server by IP address
	@param ip       the IP address to connect to
	@param port     the port to connect to
	@return returns 0 if last command is still executing, 1 success, 2 if there are no resources
 */
int ESP8266_AT_Client::connect(IPAddress ip, uint16_t port, esp8266_connect_proto_t proto){
  char host[16] = {0}; // worst case 111.111.111.111 + the null terminator = 16 characters
  snprintf(host, 15, "%d.%d.%d.%d", ip[3], ip[2], ip[1], ip[0]);
  return connect(host, port, proto);
}

int ESP8266_AT_Client::connect(uint32_t ip, uint16_t port, esp8266_connect_proto_t proto){
  char host[16] = {0}; // worst case 111.111.111.111 + the null terminator = 16 characters
  IpUint32ToString(ip, (char *) host);
  return connect(host, port, proto);
}

/** Connect to server by hostname
	@param host			the host name to resolve to an IP address and connect to
	@param port			the port to connect to
	@return returns 0 if last command is still executing, 1 success, 2 if there are no resources
 */
int ESP8266_AT_Client::connect(const char *host, uint16_t port, esp8266_connect_proto_t proto){
  // only implementing a blocking API - never return 0
  // set up an AT command and send it
  // then return whether or not it succeeded
  socket_connected = false;
  listener_started = false;

  ESP8266_DEBUG("Connecting to ", (char *) host);

  waitForIncomingDataToComplete();
  flushInput();
  
  ok_flag = false;
  error_flag = false;  
  streamWrite("AT+CIPSTART=\"");
  if(proto == ESP8266_TCP){
    streamWrite("TCP");
  }
  else if(proto == ESP8266_UDP){
    streamWrite("UDP");
  }

  streamWrite("\",\"");
  streamWrite(host);
  streamWrite("\",");
  streamWrite((uint32_t) port);
  if(proto == ESP8266_TCP){
    streamWrite(",");
    streamWrite((uint32_t) tcp_keep_alive_interval_seconds); // keep alive interval in units of seconds
  }
  streamWrite("\r\n");

  const int32_t interval = 5000;
  uint32_t current_millis = millis();
  uint32_t previous_millis = current_millis;
  boolean timeout_flag = false;
  while(!error_flag && !ok_flag && !timeout_flag){
    current_millis = millis();

    if(stream->available() > 0){
      int16_t b = streamReadChar();
      if(b > 0){
        previous_millis = current_millis;      
      }
    }

    if (current_millis - previous_millis >= interval) {
      timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
      Serial.println("PANIC7");
      printDebugWindow();
#endif
    }  
  }    

  if(ok_flag){
    socket_connected = true;
    socket_type = proto;
  }

  return ok_flag;
}

int ESP8266_AT_Client::connect(IPAddress ip, uint16_t port){
  return connect(ip, port, ESP8266_TCP);
}

int ESP8266_AT_Client::connect(const char *host, uint16_t port){
  return connect(host, port, ESP8266_TCP);
}

boolean ESP8266_AT_Client::connectUDP(uint32_t ip, uint16_t port){
  boolean ret = false;
  if(connect(ip, port, ESP8266_UDP)){
    ret = true;
  }
  return ret;
}

boolean ESP8266_AT_Client::connectUDP(const char *host, uint16_t port){
  boolean ret = false;
  if(connect(host, port, ESP8266_UDP)){
    ret = true;
  }
  return ret;
}

boolean ESP8266_AT_Client::setMacAddress(uint8_t * mac_address){
  
  boolean ret = false;
  char mac_str[18] = {0};
  macArrayToString(mac_address, (char *) mac_str);
  boolean timeout_flag = false;
  
  if(strlen(mac_str) == 17){ // e.g. 00:04:4a:23:11:7b
    waitForIncomingDataToComplete();    
    flushInput();

    ok_flag = false;
    error_flag = false;
    streamWrite("AT+CIPSTAMAC_CUR=\"");
    streamWrite(mac_str);
    streamWrite("\"\r\n");

    const int32_t interval = 5000;
    uint32_t current_millis = millis();
    uint32_t previous_millis = current_millis;    
    while(!error_flag && !ok_flag && !timeout_flag){
      current_millis = millis();

      if(stream->available() > 0){
        int16_t b = streamReadChar();
        if(b > 0){
          previous_millis = current_millis;      
        }
      }

      if (current_millis - previous_millis >= interval) {
        timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)
        Serial.println("PANIC7");
        printDebugWindow();
#endif
      }  

      if(ok_flag){
        ret = true;
      }
    }    
  }

  return ret;
}

boolean ESP8266_AT_Client::listen(uint16_t port){

  boolean ret = false;
  flushInput();

  // setup tcp server
  waitForIncomingDataToComplete();

  ok_flag = false;
  error_flag = false;  
  streamWrite("AT+CIPMUX=1");
  streamWrite("\r\n");

  const int32_t interval = 500;
  uint32_t current_millis = millis();
  uint32_t previous_millis = current_millis;
  boolean timeout_flag = false;
  while(!error_flag && !ok_flag && !timeout_flag){
    current_millis = millis();

    if(stream->available() > 0){
      int16_t b = streamReadChar();
      if(b > 0){
        previous_millis = current_millis;      
      }          
    }

    if (current_millis - previous_millis >= interval) {
      timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
      Serial.println("PANIC11");
      printDebugWindow();
#endif
    }
  }  

  if(ok_flag){ // so far so good

    delay(100);
    flushInput();
    ret = false;

    waitForIncomingDataToComplete();

    ok_flag = false;
    error_flag = false;    
    streamWrite("AT+CIPSERVER=1,");
    streamWrite((uint32_t) port);
    streamWrite("\r\n");

    current_millis = millis();
    previous_millis = current_millis;
    timeout_flag = false;
    while(!error_flag && !ok_flag && !timeout_flag){
      current_millis = millis();

      if(stream->available() > 0){
        int16_t b = streamReadChar();
        if(b > 0){
          previous_millis = current_millis;      
        }          
      }

      if (current_millis - previous_millis >= interval) {
        timeout_flag = true;
  #if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
        Serial.println("PANIC14");
        printDebugWindow();
  #endif
      }

      if(ok_flag){
        ret = true;
      }
    }
  }

  return ret;
}

boolean ESP8266_AT_Client::configureSoftAP(const char *ssid, const char *pwd, uint8_t channel, uint8_t sec){

  waitForIncomingDataToComplete();
  flushInput();

  ok_flag = false;
  error_flag = false;  
  streamWrite("AT+CWSAP_CUR=\"");
  streamWrite(ssid);
  streamWrite("\",\"");
  streamWrite(pwd);
  streamWrite("\",");
  streamWrite((uint32_t) channel);
  streamWrite(",");
  streamWrite((uint32_t) sec);
  streamWrite("\r\n");

  const int32_t interval = 10000;
  uint32_t current_millis = millis();
  uint32_t previous_millis = current_millis;
  boolean timeout_flag = false;
  while(!error_flag && !ok_flag && !timeout_flag){
    current_millis = millis();

    if(stream->available() > 0){
      int16_t b = streamReadChar();
      if(b > 0){
        previous_millis = current_millis;      
      }          
    }

    if (current_millis - previous_millis >= interval) {
      timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
      Serial.println("PANIC15");
      printDebugWindow();
#endif
    }
  }

  return ok_flag;
}

// 0 : disable sleep mode
// 1 : light-sleep mode
// 2 : modem-sleep mode
boolean ESP8266_AT_Client::sleep(uint8_t mode){

  waitForIncomingDataToComplete();

  ok_flag = false;
  error_flag = false;  
  streamWrite("AT+SLEEP=");
  streamWrite((uint32_t) mode);
  streamWrite("\r\n");

  const int32_t interval = 100;
  uint32_t current_millis = millis();
  uint32_t previous_millis = current_millis;
  boolean timeout_flag = false;
  while(!error_flag && !ok_flag && !timeout_flag){
    current_millis = millis();

    if(stream->available() > 0){
      int16_t b = streamReadChar();
      if(b > 0){
        previous_millis = current_millis;      
      }          
    }

    if (current_millis - previous_millis >= interval) {
      timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
      Serial.println("PANIC17");
      printDebugWindow();
#endif
    }
  }
  
  return ok_flag;
}

// note dnsServer is currently ignored as there is no direct support for it in the AT command set, afaict
// at any rate, we are currently *emulating* DNS in this library, not actually sending explicit DNS requests to a name server
boolean ESP8266_AT_Client::setStaticIPAddress(uint32_t ipAddress, uint32_t netMask, uint32_t defaultGateway, uint32_t dnsServer){  
  char ip_str[16] = {0};
  char netMask_str[16] = {0};
  char defaultGateway_str[16] = {0};
  char dnsServer_str[16] = {0};

  IpUint32ToString(ipAddress, (char *) ip_str);
  IpUint32ToString(netMask, (char *) netMask_str);
  IpUint32ToString(defaultGateway, (char *) defaultGateway_str);
  IpUint32ToString(dnsServer, (char *) dnsServer_str);

  waitForIncomingDataToComplete();

  ok_flag = false;
  error_flag = false;  
  streamWrite("AT+CIPSTA_CUR=\"");
  streamWrite((char *) ip_str);
  streamWrite("\",\"");
  streamWrite((char *) defaultGateway_str);
  streamWrite("\",\"");
  streamWrite((char *) netMask_str);
  streamWrite("\"\r\n");

  const int32_t interval = 1000;
  uint32_t current_millis = millis();
  uint32_t previous_millis = current_millis;
  boolean timeout_flag = false;
  while(!error_flag && !ok_flag && !timeout_flag){
    current_millis = millis();

    if(stream->available() > 0){
      int16_t b = streamReadChar();
      if(b > 0){
        previous_millis = current_millis;      
      }          
    }

    if (current_millis - previous_millis >= interval) {
      timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
      Serial.println("PANIC18");
      printDebugWindow();
#endif
    }
  }

  return ok_flag;

}

boolean ESP8266_AT_Client::setDHCP(void){
  boolean ret = false;

  waitForIncomingDataToComplete();

  ok_flag = false;
  error_flag = false;  
  streamWrite("AT+CWDHCP_CUR=1,1\r\n");

  const int32_t interval = 2000;
  uint32_t current_millis = millis();
  uint32_t previous_millis = current_millis;
  boolean timeout_flag = false;
  while(!error_flag && !ok_flag && !timeout_flag){
    current_millis = millis();

    if(stream->available() > 0){
      int16_t b = streamReadChar();
      if(b > 0){
        previous_millis = current_millis;      
      }          
    }

    if (current_millis - previous_millis >= interval) {
      timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
      Serial.println("PANIC19");
      printDebugWindow();
#endif
    }
  }

  return ok_flag;
}

/** Write a character in request
	@param c			Character to write
	@return 1 if a character was sent 0 otherwise
 */
size_t ESP8266_AT_Client::write(uint8_t c){
  uint8_t buf[1] = {c};
  return write(buf, 1);
}

/** Write a characters buffer in request
	@param buf    a buffer of bytes to send
	@param sz     the number of bytes in the buffer
	@return sz if buf was sent 0 otherwise
 */
size_t ESP8266_AT_Client::write(const uint8_t *buf, size_t sz){
  size_t ret = 0;
  boolean timeout_flag = false;
  boolean ok_to_exit = false;
  boolean got_ok = false;
  boolean got_sendok = false;  
  boolean wroteData = false;
  boolean gotArrow = false;

  int32_t interval = 2000;
  uint32_t current_millis = millis();
  uint32_t previous_millis = current_millis;

  // expect to get ">"
  // then send the bytes
  // then expect to get SEND OK

  waitForIncomingDataToComplete();
  flushInput();

  ok_flag = false;
  send_ok_flag = false;
  error_flag = false;    

  streamWrite("AT+CIPSEND=");
  if(listener_started){
    // TODO: this assumes the link id is zero, and so only supports one connection
    streamWrite("0,");
  }
  streamWrite((uint32_t) sz);
  streamWrite("\r\n");

  while(!error_flag && !ok_to_exit && !timeout_flag){
    current_millis = millis();

    if(stream->available() > 0){
      int16_t b = streamReadChar();
      if(b > 0){
        previous_millis = current_millis;      
        if(b == '>'){
          gotArrow = true;            
        }
      }

      if(got_ok && !wroteData && gotArrow){ // this is the trigger to write bytes to the output stream
        ret = streamWrite(buf, sz); // pass it along
        wroteData = true;
        // Serial.println("WROTE DATA");
      }      
    }

    if(ok_flag){      
      got_ok = true;      
      // Serial.println("GOT OK");
    }
    else if(send_ok_flag){
      got_sendok = true;
      // Serial.println("GOT SEND OK");
    }

    if (current_millis - previous_millis >= interval) {
      timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
      Serial.println("PANIC16");
      printDebugWindow();
#endif
    }
    else{
      ok_to_exit = got_ok && got_sendok && wroteData;
    }
  }
  
  // NOTE: this next bit is 'magic bullet'
  //       sometimes sending doesn't pick up the expected event sequence
  //       so... if you go through all that and timeout without writing
  //       just go ahead and write anyway and hope for the best
  //       and this actually seems to work reliably

  // don't leave the ESP hanging waiting for data
  if((ret < sz) && timeout_flag){
    streamWrite(buf + ret, sz - ret); 
    ret = 0; // mark a failure
    // so if ret = 0 it does streamWrite(buf, sz);
    // if ret = sz - 1 it does streamWrite(&buf[sz-1], 1);
    // and everything in between
  }

  // Serial.print("RET=");
  // Serial.println(ret);
  // Serial.println();

  // datasheet actually says: 
  // Enter transparent transmission, with a 20-ms
  // interval between each packet, and a maximum of
  // 2048 bytes per packet.
  
  // delay(25); // minimum 20ms, lets be generous
  // but we can't _actually_ do delay, because 
  // our send _may_ have inspired a flood of incoming
  // packet data, i.e. if we ar downloading a file
  // so we need to handle incoming traffic while we wait
  interval = 25;
  current_millis = millis();
  previous_millis = current_millis;
  while(current_millis - previous_millis < interval){
    current_millis = millis();
    flushInput(); // this should handle incoming traffic
  }

  // should return the number of bytes written
  return ret;
}

/** Check if there is data pending receipt
	@return 1 if exists, 0 if not exists
 */
int ESP8266_AT_Client::available(){
  // an intent to read() is always preceded by a call to available(),
  // if the caller knows what is good for them,
  // so this is where we need to perform asynchronous receipt of +IPD data  
  flushInput(); // obviously nobody is waiting for AT command responses
                // so consume them bytes

  return num_consumed_bytes_in_input_buffer;
}

/** Read a character from stream
	@return character
 */
int ESP8266_AT_Client::read(){
  return readFromInputBuffer();
}

/** Read from stream and copy size specified to buffer
	@param buf			Buffer
	@param size			Buffer size
	@return bytes read
 */
int ESP8266_AT_Client::read(uint8_t *buf, size_t size){
  int num_bytes_to_read = size;
  if(num_consumed_bytes_in_input_buffer < num_bytes_to_read){
    num_bytes_to_read = num_consumed_bytes_in_input_buffer;
  }

  for(int ii = 0; ii < num_bytes_to_read; ii++){
    *buf++ = readFromInputBuffer();
  }

  return num_bytes_to_read;
}

/** Read a character from response buffer but does not move the pointer.
	@returns input_buffer[input_buffer_read_idx];
 */
int ESP8266_AT_Client::peek(){
  return input_buffer[input_buffer_read_idx];
}

/** Flush response buffer
 */
void ESP8266_AT_Client::flush(){
  // Serial.println("FLUSHED");
  stream->flush();
}

/** Stop client
 */
void ESP8266_AT_Client::stop(){  
  if(socket_connected || (socket_type == ESP8266_UDP) || listener_started){

    // set up an AT command and send it
    // then return whether or not it succeeded

    waitForIncomingDataToComplete();
    flushInput();

    ok_flag = false;
    error_flag = false;
    streamWrite("AT+CIPCLOSE");
    if(listener_started){
      streamWrite("=0"); //TODO: assumes target is link id 0
    }
    streamWrite("\r\n");

    const int32_t interval = 5000;
    uint32_t current_millis = millis();
    uint32_t previous_millis = current_millis;
    boolean timeout_flag = false;
    while(!error_flag && !ok_flag && !timeout_flag){
      current_millis = millis();

      if(stream->available() > 0){
        int16_t b = streamReadChar();
        if(b > 0){
          previous_millis = current_millis;      
        } 
      }         

      if (current_millis - previous_millis >= interval) {
        timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
        Serial.println("PANIC20");
        printDebugWindow();
#endif
      }
    }
  }  

  // and drop all the remaining unread user buffer data
  while(num_consumed_bytes_in_input_buffer > 0){
    readFromInputBuffer();
  }

  socket_connected = false;

}

/** Check if connected to server
	@return 1 if connected
 */

uint8_t ESP8266_AT_Client::connected(){
  return connected(false);
}

// returns true *only* if the requested SSID is found
// if multiples of the SSID are found, the one with the highest RSSI is returned
boolean ESP8266_AT_Client::scanForAccessPoint(char * ssid, ap_scan_result_t * result, uint8_t * num_results_found, uint32_t timeout_ms){
  boolean ret = false;
  int8_t max_rssi = -128;

  waitForIncomingDataToComplete();

  ok_flag = false;
  error_flag = false;  
  streamWrite("AT+CWLAP");
  streamWrite("\r\n");

  uint8_t match_index = 0xFF;
  char line[128] = {0};
  uint16_t line_idx = 0;
  boolean inside_result = false;
  boolean inside_quotes = false;
  uint8_t result_number = 0;

  const int32_t interval = timeout_ms;
  uint32_t current_millis = millis();
  uint32_t previous_millis = current_millis;
  boolean timeout_flag = false;

  // anytime you encounter a '(' start buffering a line
  // anytime you encounter a ')' process the buffered line
  // exit when you get OK and you aren't inside_result
  while(!timeout_flag){
    current_millis = millis();

    if(stream->available() > 0){
      int16_t b = streamReadChar();
      if(b > 0){
        previous_millis = current_millis;
        if(!inside_result){
          if(b == '('){
            inside_quotes = false;
            inside_result = true;
            line_idx = 0;
            line[0] = 0;      
          }
          else if(ok_flag || error_flag){ // you got OK or ERROR and you're not in a scan result
            break;                        // either we got the result or we didn't, but in any case we're done
          }
        }
        else if(inside_result){
          if(!inside_quotes && (b == ')')){
            inside_result = false;
            // process the line
            ap_scan_result_t res = {0};
            parseScanResult(&res, line);
            result_number++;
            if((strcmp(&(res.ssid[0]), ssid) == 0) && (res.rssi > max_rssi)){
              *result = res;
              max_rssi = res.rssi;
              ret = true;
            }  
          }
          else {
            if(b == '"'){
              inside_quotes = !inside_quotes;
            }

            // if there is still space in the buffer enqueue b
            if(line_idx < 127){ // line[127] must always be zero, and we're about to write two consecutive locations
              line[line_idx++] = b;  // add a character, advance the write index
              line[line_idx] = '\0'; // and enforce a null terminator
            }
          }
        }
        // otherwise you can ignore the character
        // because you are not inside results
        // and you have not just seen a '('
      }          
    }

    if (current_millis - previous_millis >= interval) {
      timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
      Serial.println("PANIC21");
      printDebugWindow();
#endif
    }

  }

  *num_results_found = result_number;

  return ret;
}

// returns true if *any* SSIDs are found
boolean ESP8266_AT_Client::scanAccessPoints(ap_scan_result_t * results, uint8_t max_num_results, uint8_t * num_results_found, uint32_t timeout_ms){  
  boolean ret = false;
  int8_t max_rssi = -128;

  waitForIncomingDataToComplete();

  ok_flag = false;
  error_flag = false;  
  streamWrite("AT+CWLAP");
  streamWrite("\r\n");

  uint8_t match_index = 0xFF;
  char line[128] = {0};
  uint16_t line_idx = 0;
  boolean inside_result = false;
  boolean inside_quotes = false;
  uint8_t result_number = 0;

  const int32_t interval = timeout_ms;
  uint32_t current_millis = millis();
  uint32_t previous_millis = current_millis;
  boolean timeout_flag = false;

  // anytime you encounter a '(' start buffering a line
  // anytime you encounter a ')' process the buffered line
  // exit when you get OK and you aren't inside_result
  while(!timeout_flag){
    current_millis = millis();

    if(stream->available() > 0){
      int16_t b = streamReadChar();
      if(b > 0){
        previous_millis = current_millis;
        if(!inside_result){
          if(b == '('){
            inside_quotes = false;
            inside_result = true;
            line_idx = 0;
            line[0] = 0;      
          }
          else if(ok_flag || error_flag){ // you got OK or ERROR and you're not in a scan result
                                          // either we got the result or we didn't, but in any case we're done
            ret = ok_flag;                // if we got 'OK' then the result is good
            break;
          }
        }
        else if(inside_result){
          if(!inside_quotes && (b == ')')){
            inside_result = false;
            if(result_number < max_num_results){
              // process the line, but only if there's space for the result          
              parseScanResult(&(results[result_number]), line);            
            }
            result_number++;
          }
          else {
            if(b == '"'){
              inside_quotes = !inside_quotes;
            }

            // if there is still space in the buffer enqueue b
            if(line_idx < 127){ // line[127] must always be zero, and we're about to write two consecutive locations
              line[line_idx++] = b;  // add a character, advance the write index
              line[line_idx] = '\0'; // and enforce a null terminator
            }
          }
        }
        // otherwise you can ignore the character
        // because you are not inside results
        // and you have not just seen a '('
      }      
    }    

    if (current_millis - previous_millis >= interval) {
      timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
      Serial.println("PANIC22");
      printDebugWindow();
#endif
    }

  }

  *num_results_found = result_number;

  return ret;
}

void ESP8266_AT_Client::parseScanResult(ap_scan_result_t * result, char * line){
  // tokenize the line on commas
  result->security = 0;
  memset(&(result->ssid[0]), 0, 32);
  result->rssi = -128;
  memset(&(result->mac[0]), 0, 6);

  char * token = strtok(line, ",");
  uint8_t token_number = 0;
  uint8_t len = 0;
  while(token != NULL){
    switch(token_number){
    case 0:
      result->security = atoi(token);
      break;
    case 1:
      // ssid is quoted
      len = strlen(token);
      if((len <= 32) && (token[0] == '\"') && (token[len - 1] == '\"')){
        token[len - 1] = NULL;
        strncpy(&(result->ssid[0]), &(token[1]), 33);
      }
      break;
    case 2:
      result->rssi = atoi(token);
      break;
    case 3:
      // mac address is quoted
      len = strlen(token);
      if((len <= 32) && (token[0] == '\"') && (token[len - 1] == '\"')){
        token[len - 1] = NULL;
        stringToMacArray(&(token[1]), &(result->mac[0]));
        break;
      }
      break;
    default:
      break;
    }
    token = strtok(NULL, ",");
    token_number++;
  }
}

boolean ESP8266_AT_Client::getRemoteIp(uint32_t * ip){
  boolean ret = false;
  uint8_t num_quotes = 0;
  char remote_ip_str[16] = {0};
  uint8_t remote_ip_str_idx = 0;

  waitForIncomingDataToComplete();

  ok_flag = false;
  error_flag = false;  
  streamWrite("AT+CIPSTATUS");
  streamWrite("\r\n");

  const int32_t interval = 500;
  uint32_t current_millis = millis();
  uint32_t previous_millis = current_millis;
  boolean timeout_flag = false;
  while(!error_flag && !ok_flag && !timeout_flag){
    current_millis = millis();

    if(stream->available() > 0){
      int16_t b = streamReadChar();
      if(b > 0){
        previous_millis = current_millis;      
        if(b == '"'){
          num_quotes++;
        }
        
        if((num_quotes == 3) && (b != '"')){ // don't buffer the leading '"'
          if(remote_ip_str_idx < 15){
            remote_ip_str[remote_ip_str_idx++] = b;
            remote_ip_str[remote_ip_str_idx] = '\0';
          }
        }
        else if(num_quotes == 4){
          num_quotes++;
          ret = stringToIpUint32((char *) remote_ip_str, ip);
        }
        
      }    
    }

    if (current_millis - previous_millis >= interval) {
      timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
      Serial.println("PANIC23");
      printDebugWindow();
#endif
    }
  }  

  return ret;
}

uint8_t ESP8266_AT_Client::connected(boolean actively_check){
  uint8_t ret = 1; // assume we are connected
  char numeric_value = 0;

  flushInput(); 

  if(actively_check){ // things seem go bad if you do this in a tight loop
    // set up an AT command and send it
    // then return whether or not it succeeded
    // you have to get "STATUS:" and a numeric code ('4' means disconnected)
    // then you have *may* get "+CIPSTATUS:"
    // then you get "OK"

    ok_flag = false;
    error_flag = false;
    streamWrite("AT+CIPSTATUS");
    streamWrite("\r\n");

    const int32_t interval = 200;
    uint32_t current_millis = millis();
    uint32_t previous_millis = current_millis;
    boolean timeout_flag = false;
    while(!error_flag && !ok_flag && !timeout_flag){
      current_millis = millis();

      if(stream->available() > 0){
        int16_t b = streamReadChar();
        if(b > 0){
          previous_millis = current_millis;      
          if(b == '4'){ // STATUS:4 means disconnected
            socket_connected = false;
            ret = 0;
          }      
        }
      }

      if (current_millis - previous_millis >= interval) {
        timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
        Serial.println("PANIC24");
        printDebugWindow();
#endif
      }  
    }  
  }
  else{
    ret = socket_connected;
  }

  return ret;
}

uint8_t ESP8266_AT_Client::connectedToNetwork(void){  
  // TODO: this should just be goverened by a state variable
  // and the updatePlusIpdState state machine should basically just watch
  // for WIFI	DISCONNECT to clear the state variable rather than 
  // actively interrogating the ESP for this information

  return wifi_is_connected;
}

boolean ESP8266_AT_Client::getIPAddress(char * ip_str, char * gateway_str, char * netmask_str){
  char * write_ptr = 0;
  char write_idx = 0;
  char num_quotes = 0;
  boolean wrote_ipstr = false;
  boolean wrote_gateway_str = false;
  boolean wrote_netmask_str = false;
  boolean ok_to_exit = false;
  waitForIncomingDataToComplete();
  flushInput();

  ok_flag = false;
  error_flag = false;
  streamWrite("AT+CIPSTA?");
  streamWrite("\r\n");

  const int32_t interval = 1000;
  uint32_t current_millis = millis();
  uint32_t previous_millis = current_millis;
  boolean timeout_flag = false;
  while(!error_flag && !ok_to_exit && !timeout_flag){

    current_millis = millis();
    
    if(stream->available() > 0){
      int16_t b = streamReadChar();
      if(b > 0){
        previous_millis = current_millis; 

        if(b == '"'){
          num_quotes++;
          switch(num_quotes){
          case 1: write_ptr = ip_str; write_idx = 0; break;
          case 2: wrote_ipstr = true; break;
          case 3: write_ptr = gateway_str;  write_idx = 0; break;
          case 4: wrote_gateway_str = true; break;
          case 5: write_ptr = netmask_str;  write_idx = 0; break;
          case 6: wrote_netmask_str = true; break;
          default: write_ptr = NULL; write_idx = 0;
          }                  
        }
        else if(write_ptr != NULL){ // write pointer was set and b is != '"'     
          // IP addresses can't be longer than "255.255.255.255" (15 characters)
          // so write_idx should never obtain a value > 15
          if(write_idx < 15){
            write_ptr[write_idx++] = b;  // NOTE: this is _still_ dangerous if the passed pointers don't have enough space
            write_ptr[write_idx] = '\0'; //enforce NULL terminate
          }
        }
      }  
    }      

    if (current_millis - previous_millis >= interval) {
      timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
      Serial.println("PANIC30");
      printDebugWindow();
#endif
    }
    else{
      ok_to_exit = ok_flag && wrote_ipstr && wrote_gateway_str && wrote_netmask_str;      
    }
  }  

  return ok_to_exit;
}

uint32_t ESP8266_AT_Client::IpArrayToIpUint32(uint8_t * ip){
  uint32_t ret = 0;
  ret |= ((uint32_t) ip[0]) << 24;
  ret |= ((uint32_t) ip[1]) << 16;
  ret |= ((uint32_t) ip[2]) << 8;
  ret |= ((uint32_t) ip[3]) << 0;
  return ret;
}

void ESP8266_AT_Client::IpUint32ToArray(uint32_t ip, uint8_t * ip_arr){
  ip_arr[0] = (ip >> 24) & 0xff;
  ip_arr[1] = (ip >> 16) & 0xff;
  ip_arr[2] = (ip >> 8) & 0xff;
  ip_arr[3] = (ip >> 0) & 0xff;
}

boolean ESP8266_AT_Client::stringToIpArray(char * str, uint8_t * ip){
  uint32_t ip_u32 = 0;
  boolean ret = false;
  if(stringToIpUint32(str, &ip_u32)){
    IpUint32ToArray(ip_u32, ip);
    ret = true;
  }
  return ret;
}

void ESP8266_AT_Client::IpArrayToString(uint8_t * ip, char * tgt){
  uint32_t ip_u32 = IpArrayToIpUint32(ip);
  IpUint32ToString(ip_u32, tgt);
}

void ESP8266_AT_Client::IpUint32ToString(uint32_t ip, char * tgt){
  sprintf(tgt, "%d.%d.%d.%d",
    (uint8_t) (ip >> 24),
    (uint8_t) (ip >> 16),
    (uint8_t) (ip >> 8),
    (uint8_t) (ip >> 0));
}

boolean ESP8266_AT_Client::stringToIpUint32(char * str, uint32_t * ip){
  char * token = strtok(str, ".");
  uint8_t num_tokens = 0;
  uint32_t ip_address = 0;
  boolean ret = false;

  while(token != NULL){
    num_tokens++;

    if(num_tokens > 4){
      break;
    }

    if(num_tokens > 1){
      ip_address <<= 8;
    }
    ip_address |= atoi(token) & 0xFF;

    token = strtok(NULL, ".");
  }

  if(num_tokens == 4){
    ret = true;
    *ip = ip_address;
  }

  return ret;
}

void ESP8266_AT_Client::macArrayToString(uint8_t * mac, char * tgt){
  sprintf(tgt, "%02X:%02X:%02X:%02X:%02X:%02X",
    mac[0],
    mac[1],
    mac[2],
    mac[3],
    mac[4],
    mac[5]);
}

boolean ESP8266_AT_Client::stringToMacArray(char * str, uint8_t * mac){
  boolean ret = false;
  char * token = strtok(str, ":");
  uint8_t num_tokens = 0;
  uint32_t ip_address = 0;
  char local_mac[6] = {0};

  while(token != NULL){
    num_tokens++;
    if(num_tokens > 6){
      break;
    }

    if(strlen(token) == 2){
      char * temp = NULL;
      uint32_t octet = strtoul((char *) token, &temp, 16);
      if (*temp == '\0'){
        local_mac[num_tokens-1] = octet;
      }
      else{
        break;
      }
    }
    else{
      break;
    }

    token = strtok(NULL, ":");
  }

  if(num_tokens == 6){
    ret = true;
    memcpy(mac, local_mac, 6);
  }
  return ret;
}

boolean ESP8266_AT_Client::getIPAddress(uint32_t * ip, uint32_t * gateway, uint32_t * netmask){
  boolean ret = false;
  char ip_str[16] = {0};
  char gateway_str[16] = {0};
  char netmask_str[16] = {0};
  ret = getIPAddress((char *) &(ip_str[0]), (char *) &(gateway_str[0]), (char *) &(netmask_str[0]));
  if(ret){
    stringToIpUint32((char *) &(ip_str[0]), ip);
    stringToIpUint32((char *) &(gateway_str[0]), gateway);
    stringToIpUint32((char *) &(netmask_str[0]), netmask);
  }

  return ret;
}

boolean ESP8266_AT_Client::getMacAddress(char * mac_str){  
  uint8_t num_commas = 0;  
  uint8_t num_quotes = 0;
  uint8_t write_idx = 0;

  waitForIncomingDataToComplete();
  flushInput();

  ok_flag = false;
  error_flag = false;  
  streamWrite("AT+CIFSR");
  streamWrite("\r\n");

  // wait for ok or error or timeout  
  const int32_t interval = 100;
  uint32_t current_millis = millis();
  uint32_t previous_millis = current_millis;
  boolean timeout_flag = false;
  while(!error_flag && !ok_flag && !timeout_flag){
    current_millis = millis();

    if(stream->available() > 0){
      int16_t b = streamReadChar();      
      if(b > 0){
        previous_millis = current_millis;
        if(b == ','){
          num_commas++;
        }
        else if(b == '"'){
          num_quotes++;
        }

        if(num_commas == 2){ // station MAC is the last one reported (i.e. after the 1st comma)
          if((b != '"') && (num_quotes == 3)){ // betweeb 3rd and 4th quotes
            // worst case mac address is FF:FF:FF:FF:FF:FF (17 characters)
            if(write_idx < 17){ // [16] is the last viable write location
              mac_str[write_idx++] = b;
              mac_str[write_idx] = '\0';
            }
          }
        }         
      }
    }

    if (current_millis - previous_millis >= interval) {
      timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
      Serial.println("PANIC25");
      printDebugWindow();
#endif
    }
  }  

  return ok_flag;
}

boolean ESP8266_AT_Client::getMacAddress(uint8_t * mac){
  boolean ret = false;
  char tmp[18] = {0};
  if(getMacAddress((char *) tmp)){
    ret = stringToMacArray((char *) tmp, mac);
  }

  return ret;
}

boolean ESP8266_AT_Client::getHostByName(const char *hostname, uint32_t *ip, uint32_t timeout_ms){
  // connect to host name on port 7
  // then read the connection status to get teh remote IPAddress
  boolean ret = false;

  socket_type = ESP8266_UDP;

  waitForIncomingDataToComplete();  
  flushInput();
  
  ok_flag = false;
  error_flag = false;
  streamWrite("AT+CIPSTART=\"UDP\",\"");
  streamWrite(hostname);
  streamWrite("\",7\r\n");

  const int32_t interval = timeout_ms;
  uint32_t current_millis = millis();
  uint32_t previous_millis = current_millis;
  boolean timeout_flag = false;
  while(!error_flag && !ok_flag && !timeout_flag){
    current_millis = millis();
  
    if(stream->available() > 0){
      int16_t b = streamReadChar();
      if(b > 0){
        previous_millis = current_millis;      
      }          
    }

    if (current_millis - previous_millis >= interval) {
      timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
      Serial.println("PANIC26");
      printDebugWindow();
#endif
    }  
  }  

  if(ok_flag){
    uint32_t remote_ip = 0;
    ret = getRemoteIp(&remote_ip);
    if(ret){
      *ip = remote_ip;
    }
  }

  stop();

  return ret;
}

ESP8266_AT_Client::operator bool(){
  return (connected()==1);
}

// writes c to the write pointer in the input buffer
// otherwise the write pointer is advanced
boolean ESP8266_AT_Client::writeToInputBuffer(uint8_t c){
  if(num_free_bytes_in_input_buffer > 0){
    input_buffer[input_buffer_write_idx++] = c; // write the value to the buffer
    
    // handle wrap around
    if(input_buffer_write_idx >= input_buffer_length){
      input_buffer_write_idx = 0;
    }

    num_consumed_bytes_in_input_buffer++;
    num_free_bytes_in_input_buffer--;

    return true;
  }
  else{
    ESP8266_DEBUG("Input Buffer Underflow!");
    return false;
  }
}

// returns the character at the current read pointer
// there are no bytes available to read
// the read pointer is not advanced
// otherwise the read pointer is advanced
int16_t ESP8266_AT_Client::readFromInputBuffer(void){
  if(num_consumed_bytes_in_input_buffer > 0){    
    uint8_t ret = input_buffer[input_buffer_read_idx++];

    // handle wrap around
    if(input_buffer_read_idx >= input_buffer_length){
      input_buffer_read_idx = 0;
    }

    num_consumed_bytes_in_input_buffer--;
    num_free_bytes_in_input_buffer++;

    return ret; // returns the value extracted from the input buffer
  }

  return ((int16_t) -1); // returns -1 if you try and read and there is nothing available
}

void ESP8266_AT_Client::flushInput(){

  uint16_t bytesAvailable = stream->available();

  if(bytesAvailable > ESP8266_AT_Client::bytesAvailableMax){
    ESP8266_AT_Client::bytesAvailableMax = bytesAvailable;
  }      

#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)           
  if(bytesAvailable > 50){
    // TODO: complain loudly if this happens 
    Serial.println("PANIC12");      
    printDebugWindow();
  } 
#endif

  if(bytesAvailable > 0){  
    while(stream->available() > 0){    
      int16_t chr = streamReadChar();

  #ifdef ESP8266_AT_CLIENT_DEBUG_ECHO_EVERYTHING
      if(debugStream != NULL && debugEnabled) debugStream->println((uint8_t) chr, HEX); // echo the received characters to the Serial Monitor
  #endif
    }
  }
}

// 1 is Station Mode
// 2 is SoftAP Mode
// 3 is SoftAP + Station Mode
boolean ESP8266_AT_Client::setNetworkMode(uint8_t mode){
  boolean ret = false;

  waitForIncomingDataToComplete();
  flushInput();

  ok_flag = false;
  error_flag = false;
  streamWrite("AT+CWMODE_CUR=");
  streamWrite((uint32_t) mode);
  streamWrite("\r\n");

  const int32_t interval = 1000;
  uint32_t current_millis = millis();
  uint32_t previous_millis = current_millis;
  boolean timeout_flag = false;
  while(!error_flag && !ok_flag && !timeout_flag){
    current_millis = millis();

    if(stream->available() > 0){
      int16_t b = streamReadChar();
      if(b > 0){
        previous_millis = current_millis;      
      }          
    }

    if (current_millis - previous_millis >= interval) {
      timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
      Serial.println("PANIC27");
      printDebugWindow();
#endif
    }  
  }

  ret = ok_flag;

  flushInput();
  
  return ret;
}

boolean ESP8266_AT_Client::connectToNetwork(char * ssid, char * pwd, int32_t timeout_ms, void (*onConnect)(void), boolean permanent){
  
  boolean got_connected = false;
  boolean got_ok = false;

  waitForIncomingDataToComplete();
  flushInput();

  ok_flag = false;
  error_flag = false;
  streamWrite("AT+CWJAP");
  if(!permanent){
    streamWrite("_CUR");
  }
  streamWrite("=\"");
  streamWrite(ssid);
  streamWrite("\",\"");
  streamWrite(pwd);
  streamWrite("\"");
  streamWrite("\r\n");

  // now wait for wifi to be connected, then wait for ok
  const int32_t interval = timeout_ms;
  uint32_t current_millis = millis();
  uint32_t previous_millis = current_millis;

  boolean timeout_flag = false;  
  while(!error_flag && !timeout_flag){
    current_millis = millis();

    if(stream->available() > 0){
      int16_t b = streamReadChar();
      if(b > 0){
        previous_millis = current_millis;      
        if(wifi_is_connected){
          got_connected = true;
        }

        if(ok_flag){
          got_ok = true;
        }

        if(wifi_is_connected && got_ok){
          break;
        }
      }          
    }

    if (current_millis - previous_millis >= interval) {
      timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
      Serial.println("PANIC28");
      printDebugWindow();
#endif
    }  
  }
  
  return got_connected && got_ok;
}

boolean ESP8266_AT_Client::disconnectFromNetwork(){
  boolean ret = false;
  boolean got_ok = false;
  boolean got_disonnected = false;

  waitForIncomingDataToComplete();
  flushInput();

  ok_flag = false;
  error_flag = false;  
  streamWrite("AT+CWQAP");
  streamWrite("\r\n");

  const int32_t interval = 5000;
  uint32_t current_millis = millis();
  uint32_t previous_millis = current_millis;
  boolean timeout_flag = false;
  while(!error_flag && !timeout_flag){
    current_millis = millis();

    if(stream->available() > 0){
      int16_t b = streamReadChar();
      if(b > 0){
        previous_millis = current_millis; 
        if(!wifi_is_connected){
          got_disonnected = true;
        }

        if(ok_flag){
          got_ok = true;
        }

        if(got_disonnected && got_ok){
          break;
        }
      }          
    }

    if (current_millis - previous_millis >= interval) {
      timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
      Serial.println("PANIC29");
      printDebugWindow();
#endif
    }  
  }  

  return got_disonnected && got_ok;
}

boolean ESP8266_AT_Client::setTcpKeepAliveInterval(uint16_t _tcp_seconds){
  if(_tcp_seconds <= 7200){
    tcp_keep_alive_interval_seconds = _tcp_seconds;
    return true;
  }
  return false;
}

void ESP8266_AT_Client::ESP8266_DEBUG(char * msg){
#ifdef ESP8266_AT_CLIENT_ENABLE_DEBUG
  if((debugStream != NULL) && debugEnabled){
    debugStream->print("Debug: ");
    debugStream->println(msg);
    debugStream->flush();
  }
#endif
}

void ESP8266_AT_Client::ESP8266_DEBUG(char * msg, uint16_t value){
#ifdef ESP8266_AT_CLIENT_ENABLE_DEBUG
  if((debugStream != NULL) && debugEnabled){
    debugStream->print("Debug: ");
    debugStream->print(msg);
    debugStream->println(value);
    debugStream->flush();
  }
#endif
}

void ESP8266_AT_Client::ESP8266_DEBUG(char * msg, char * value){
#ifdef ESP8266_AT_CLIENT_ENABLE_DEBUG
  if((debugStream != NULL) && debugEnabled){
    debugStream->print("Debug: ");
    debugStream->print(msg);
    debugStream->print("\"");
    debugStream->print(value);
    debugStream->println("\"");
    debugStream->flush();
  }
#endif
}

void ESP8266_AT_Client::enableDebug(void){
#ifdef ESP8266_AT_CLIENT_ENABLE_DEBUG
  debugEnabled = true;
#endif
}

void ESP8266_AT_Client::disableDebug(void){
#ifdef ESP8266_AT_CLIENT_ENABLE_DEBUG
  debugEnabled = false;
#endif
}

// in the following example invocation pattern, timeout is obviously
// managed separately, e.g. using a blinkWithoutDelay pattern
//
//    if(esp.firmwareUpdateBegin()){
//      uint8_t status = 0xFF;
//      while(esp.firmwareUpdateStatus(&status) && !timeout){
//        if(status == 2){
//          SUCCESS!!! // maybe keep waiting until you get "ready"
//        }
//        else if(status == 3){
//          SUCCESS!!! // assuming you got 2 first, module reset is complete
//        }
//      }
//      if(timeout){
//        FAILURE!!! timeout happened
//      }
//      else if(status == 1){
//        FAILURE!!! error happened
//      }
//    }
//    else {
//      FAILURE!!! firmware update never got started
//    }

boolean ESP8266_AT_Client::firmwareUpdateBegin(){
  boolean got_plus_cipupdate_colon_1 = false;
  char last_character_received = ' ';
  char pursuit_depth = 0;

  waitForIncomingDataToComplete();
  flushInput();

  ok_flag = false;
  error_flag = false;  
  streamWrite("AT+CIUPDATE");
  streamWrite("\r\n");
  
  const int32_t interval = 5000;
  uint32_t current_millis = millis();
  uint32_t previous_millis = current_millis;
  boolean timeout_flag = false;
  while(!error_flag && !timeout_flag && !got_plus_cipupdate_colon_1){
    current_millis = millis();

    if(stream->available() > 0){
      int16_t b = streamReadChar();
      if(b > 0){        
        previous_millis = current_millis;     
        // mini-state machine looking for +CIPUPDATE:1
        //                                0123456789ab
        switch(last_character_received){
        case ' ': 
          if(b == '+'){ last_character_received = b; pursuit_depth = 1; }   
          break;     
        case '+':
          if(b == 'C'){ last_character_received = b; pursuit_depth++; }        
          else { last_character_received = ' '; }
          break;
        case 'C':
          if(b == 'I'){ last_character_received = b; pursuit_depth++; }        
          else { last_character_received = ' '; }
          break;    
        case 'I':
          if(b == 'P'){ last_character_received = b; pursuit_depth++; }        
          else { last_character_received = ' '; }
          break;  
        case 'P':
          if((b == 'U') && (pursuit_depth == 4)){ last_character_received = b; pursuit_depth++; }        
          else if((b == 'D') && (pursuit_depth == 6)){ last_character_received = b; pursuit_depth++; }        
          else { last_character_received = ' '; }
          break; 
        case 'U':
          if(b == 'P'){ last_character_received = b; pursuit_depth++; }        
          else { last_character_received = ' '; }
          break;                  
        case 'D':
          if(b == 'A'){ last_character_received = b; pursuit_depth++; }        
          else { last_character_received = ' '; }
          break;    
        case 'A':
          if(b == 'T'){ last_character_received = b; pursuit_depth++; }        
          else { last_character_received = ' '; }
          break;    
        case 'T':
          if(b == 'E'){ last_character_received = b; pursuit_depth++; }        
          else { last_character_received = ' '; }
          break;    
        case 'E':
          if(b == ':'){ last_character_received = b; pursuit_depth++; }        
          else { last_character_received = ' '; }
          break;    
        case ':':
          if(b == '1'){ 
            got_plus_cipupdate_colon_1 = true;
          }        
          last_character_received = ' '; // unconditional, goal state
          break;                                                      
        // no default state, on purpose
        }
      }          
    }

    if (current_millis - previous_millis >= interval) {
      timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
      Serial.println("PANIC31");
      printDebugWindow();
#endif
    }  
  }  
  
  return got_plus_cipupdate_colon_1;
}

// this function returns true as long as you don't get an ERROR response
// it ASSUMES you will call firmwareUpdateBegin and then call this function
// until it writes a STATUS of 2 (OK) into *status or it returns false (ERROR)
// see example invocation above
boolean ESP8266_AT_Client::firmwareUpdateStatus(uint8_t * status){
  boolean ret = true;

  const int32_t interval = 5000;
  uint32_t current_millis = millis();
  uint32_t previous_millis = current_millis;
  boolean timeout_flag = false;
  ok_flag = false;
  error_flag = false;  
  ready_flag = false;

  while(!error_flag && !ok_flag && !ready_flag && !timeout_flag){
    current_millis = millis();

    if(stream->available() > 0){
      int16_t b = streamReadChar();
      if(b > 0){
        previous_millis = current_millis;              
      }          
    }

    if (current_millis - previous_millis >= interval) {
      timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
      Serial.println("PANIC32");
      printDebugWindow();
#endif
    }  
  }  

  if(error_flag){
    ret = false;
    *status = 0x01; // got ERROR
  }  
  else if(ok_flag){
    *status = 0x02; // got OK
  }
  else if(ready_flag){
    *status = 0x03; // got ready
  }
  else{
    *status = 0xFF; // got timeout
  }

  return ret;
}

boolean ESP8266_AT_Client::getVersion(char * version){  
  uint8_t write_idx = 0;
  boolean got_colon = false;
  boolean got_paren = false;
  boolean extracted_version = false;
  boolean ok_to_exit = false;
  // normal response looks like this
  // AT version:0.50.0.0(Sep 18 2015 20:55:38)
  // SDK version:1.4.0
  // compile time:Sep 18 2015 21:46:52
  // OK
  
  waitForIncomingDataToComplete();
  flushInput();

  ok_flag = false;
  error_flag = false;
  streamWrite("AT+GMR");
  streamWrite("\r\n");

  // assume the caller has allocated space for 16 bytes
  const int32_t interval = 10000;
  uint32_t current_millis = millis();
  uint32_t previous_millis = current_millis;
  boolean timeout_flag = false;
  while(!error_flag && !ok_to_exit && !timeout_flag){
    current_millis = millis();

    if(stream->available() > 0){
      int16_t b = streamReadChar();
      if(b > 0){
        previous_millis = current_millis;   
        if(b == ':'){
          got_colon = true;
        }
        else if(b == '('){
          got_paren = true;
          extracted_version = true;
        }

        if((b != ':') && got_colon && !got_paren){
          if(write_idx < 15){ // [15] is the last available space
            version[write_idx++] = b;
            version[write_idx] = '\0';
          }
        }
      }          
    }

    if (current_millis - previous_millis >= interval) {
      timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
      Serial.println("PANIC33");
      printDebugWindow();
#endif
    }
    else {
      ok_to_exit = ok_flag && extracted_version;
    }
  }  

  return ok_to_exit;
}

boolean ESP8266_AT_Client::getVersion(uint32_t * version){
  boolean ret = false;
  uint32_t value = 0;
  char str[16] = {0};
  uint8_t num_tokens = 0;
  if(getVersion(str)){
    char * token = strtok(str, ".");

    while(token != NULL){
      num_tokens++;
      if(num_tokens > 4){
        break;
      }

      if(strlen(token) < 3){
        char * temp = NULL;
        uint32_t octet = strtoul((char *) token, &temp, 10);
        if (*temp == '\0'){
          value += octet;
        }
        else{
          break;
        }

        if(num_tokens != 4){
          value *= 100;
        }
      }
      else{
        break;
      }

      token = strtok(NULL, ".");
    }
  }

  if(num_tokens == 4){
    *version = value;
    return true;
  }
  return false;
}

boolean ESP8266_AT_Client::restoreDefault(){

  waitForIncomingDataToComplete();
  flushInput();

  ok_flag = false;
  error_flag = false;  
  streamWrite("AT+RESTORE");
  streamWrite("\r\n");
  
  const int32_t interval = 2000;
  uint32_t current_millis = millis();
  uint32_t previous_millis = current_millis;
  boolean timeout_flag = false;
  while(!error_flag && !ready_flag && !timeout_flag){
    current_millis = millis();

    if(stream->available() > 0){
      int16_t b = streamReadChar();
      if(b > 0){
        previous_millis = current_millis;         
      }          
    }

    if (current_millis - previous_millis >= interval) {
      timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
      Serial.println("PANIC34");
      printDebugWindow();
#endif
    }  
  }

  return ready_flag;
}

boolean ESP8266_AT_Client::AT(void){
  waitForIncomingDataToComplete();
  flushInput();

  ok_flag = false;
  error_flag = false;  
  streamWrite("AT");
  streamWrite("\r\n");  

  
  const int32_t interval = 100;
  uint32_t current_millis = millis();
  uint32_t previous_millis = current_millis;
  boolean timeout_flag = false;
  while(!error_flag && !ok_flag && !timeout_flag){
    current_millis = millis();

    if(stream->available() > 0){
      int16_t b = streamReadChar();
      if(b > 0){
        previous_millis = current_millis;      
      }          
    }

    if (current_millis - previous_millis >= interval) {
      timeout_flag = true;
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
      Serial.println("PANIC36");
      printDebugWindow();
#endif
    }  
  }

  return ok_flag;  
}

size_t ESP8266_AT_Client::streamWrite(const char * str){
#if defined(ESP8266_AT_CLIENT_DEBUG_OUTGOING)
  Serial.print("SEND S: ");
  Serial.println(str);
#endif  
  return streamWrite((uint8_t * ) str, strlen(str));
}

size_t ESP8266_AT_Client::streamWrite(int32_t value){
  char str[16] = {0};
  ltoa(value, str, 10);
#if defined(ESP8266_AT_CLIENT_DEBUG_OUTGOING)  
  Serial.print("SEND L: ");
  Serial.print(value);  
  Serial.print(' ');
  Serial.println(str);
#endif  
  return streamWrite((uint8_t * ) str, strlen(str));
}

size_t ESP8266_AT_Client::streamWrite(uint32_t value){
  char str[16] = {0};
  ultoa(value, str, 10);
#if defined(ESP8266_AT_CLIENT_DEBUG_OUTGOING)  
  Serial.print("SEND UL: ");
  Serial.print(value);  
  Serial.print(' ');
  Serial.println(str);  
#endif  
  return streamWrite((uint8_t * ) str, strlen(str));
}

size_t ESP8266_AT_Client::streamWrite(const uint8_t *buf, size_t sz){
  size_t bytes_written = 0;

#if defined(ESP8266_AT_CLIENT_DEBUG_OUTGOING)  
  Serial.print("SEND B: ");  
  Serial.print(sz);   
  for(uint16_t ii = 0; ii < sz; ii++){
    uint8_t b = buf[ii] & 0xff;        
    Serial.print(" 0x");
    if(b < 0x10) Serial.print('0');
    Serial.print(b, HEX);    
  }
  Serial.println();
#endif  

  while(sz > 0){
    size_t availableForWrite = streamAsPrint->availableForWrite();
    if(availableForWrite > 0){
      uint8_t value = *buf;
      
      addToDebugWriteWindow(value);
      stream->write(value);
      sz--;
      bytes_written++;
      buf++;

      // NOTE: removing the following in favor of a 20ms delay _after_ CIPSEND completes
      // if((bytes_written % 32) == 0){
      //   delay(10); // maybe don't flood the ESP8266 with bytes?
      // }
    }    
  }  

  return bytes_written;
}

int16_t ESP8266_AT_Client::streamReadChar(void){
  if(numIncomingBytesPending > 0){
    processIncomingAfterColon();
    return ((int16_t) -1); // ask the caller to try again    
  }
  else {
    if(stream->available()){
      int16_t b = stream->read() & 0xff;
      addToDebugReadWindow(b);
  #if defined(ESP8266_AT_CLIENT_DEBUG_INCOMING)
      if(debugEnabled){
        if(isprint(b) || isspace(b)) Serial.print((char) b);
        else{
          Serial.print(" 0x");
          if(b < 0x10) Serial.print('0');
          Serial.println(b, HEX);
        }
      }
  #endif

      if(updatePlusIpdState(b)){
        processIncomingUpToColon(); 
        processIncomingAfterColon(); // try immediately, but don't block
        return ((int16_t) -1);       // -1 is distinguishable from consumable data to the caller
                                     // because return type is artificially int16_t (not uint16_t)
      }
      else{
        return b;
      }    
    }
    else {
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)                   
        Serial.println("PANIC35");
        printDebugWindow();
#endif            
      return ((int16_t) -1); // don't ask the stream
    }
  }
}

// this function should be called _exactly_ when an incoming packet detection occurs
// as understood by the updatePlusIpdState function
// and therefore, it picks up processing incoming packet data _from the point_ 
// where +IPD, was received.
//
// An incoming packet looks like this: +IPD,####: *****....
// where #### is an ascii string representation of a number
// that number tells you how many bytes will be received
// then you get a colon delimeter
// then you should get exactly that many bytes reported
void ESP8266_AT_Client::processIncomingUpToColon(void){  
  // timeout after 500ms of inactivity
  uint32_t current_time = millis();    
  uint32_t previous_time = current_time;
  const int32_t timeout_interval = 500;  // signed for comparison / overflow

  // consume bytes into a local buffer until you get a ':'
  char num_bytes_str[8] = {0};
  uint8_t num_bytes_write_idx = 0;
  boolean gotColon = false;

  // treat it as an error if (num_bytes_write_idx == 8) // overflow
  // or if if (current_time - previous_time >= interval) // timeout
  // update current_time on an ongoing basis
  // assign previous_time to current_time anytime a byte is received to prolong timeout
  while(!gotColon){
    current_time = millis(); // update current_time on an ongoing basis
    uint16_t bytesAvailable = stream->available();    

    if(bytesAvailable > ESP8266_AT_Client::bytesAvailableMax){
      ESP8266_AT_Client::bytesAvailableMax = bytesAvailable;
    }     

#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)           
    if(bytesAvailable > 50){
      // TODO: complain loudly if this happens 
      Serial.println("PANIC9");      
      printDebugWindow();
    }
#endif    
    
    while((bytesAvailable > 0) && !gotColon){
      current_time = millis();          
      bytesAvailable--;
      previous_time = current_time;
      int16_t b = stream->read() & 0xff;    
      addToDebugReadWindow(b);
#if defined(ESP8266_AT_CLIENT_DEBUG_INCOMING)      
      if(debugEnabled){
        if(isprint(b) || isspace(b)) Serial.print(b);
        else{
          Serial.print(" 0x");
          if(b < 0x10) Serial.print('0');
          Serial.println(b, HEX);
        }
      }      
#endif      

      if(b == ':'){ 
        // this is the loop termination criteria
        // and ':' will _not_ be added to the num_bytes_str buffer
        gotColon = true;
      }
      else if(num_bytes_write_idx < 7){
        // num_bytes_str[15] is reserved for a null terminator
        // never allow it to be overwritten, and an attempt is made
        // treat it as an error and terminate this function        
        num_bytes_str[num_bytes_write_idx++] = b & 0xff;
        num_bytes_str[num_bytes_write_idx] = '\0';
      }
      else{
        // TODO: complain loudly if this happens
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)                   
        Serial.println("PANIC1");
        printDebugWindow();
#endif  

        return; // overflow
      }

      if (!gotColon && (current_time - previous_time >= timeout_interval)){
        // TODO: complain loudly if this happens      
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)                   
        Serial.println("PANIC2");
        printDebugWindow();
#endif

        return; // timeout
      }
    }
  }

  // if we got to here, we've received a colon and _should_ be able to parse
  // the contents of num_bytes_str into a number which we can store in num_bytes_expected
  // lets give it a shot   
  numIncomingBytesPending = atoi(num_bytes_str);

  // Serial.print(numIncomingBytesPending);
  // Serial.println(" Bytes Incoming");
  
//   char * temp;
//   numIncomingBytesPending = strtoul(num_bytes_str, &temp, 10);
//   if (*temp != '\0'){
//     // TODO: complain loudly if this happens
// #if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)               
//     Serial.println("PANIC3");
//     Serial.print("num_bytes_str was \"");
//     Serial.print(num_bytes_str);
//     Serial.println("\"");
//     for(uint16_t ii = 0; ii < strlen(num_bytes_str); ii++){
//       uint8_t b = num_bytes_str[ii];
//       if(isprint(b) || isspace(b)){
//         Serial.print((char) b);
//       }
//       else{
//         Serial.print(" 0x");
//         if(b < 0x10) Serial.print('0');
//         Serial.print(b, HEX);
//       }
//     }
//     printDebugWindow();
// #endif    
//     numIncomingBytesPending = 0;
//     return; // failed to parse length
//   }
}

// this function should be the only other one besides streamReadChar that
// ever calls stream->read()
// returns true if any bytes were consumed
boolean ESP8266_AT_Client::processIncomingAfterColon(void){
  // the job of this function is to read bytes from the hardware serial buffer
  // if and only if numIncomingBytesPending > 0
  // it should return as soon as possible
  boolean ret = false;
  if(numIncomingBytesPending > 0){

    uint16_t bytesAvailable = stream->available();

    if(bytesAvailable > ESP8266_AT_Client::bytesAvailableMax){
      ESP8266_AT_Client::bytesAvailableMax = bytesAvailable;
    }     

#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)      
    if(bytesAvailable > 50){
      // TODO: complain loudly if this happens 
      Serial.println("PANIC10");      
      printDebugWindow();
    }
#endif    

    while(bytesAvailable > 0){      
  
      int16_t b = stream->read() & 0xff;
      addToDebugReadWindow(b);

      bytesAvailable--;
      numIncomingBytesPending--;
      ret = true; // true we read some bytes from stream

#if defined(ESP8266_AT_CLIENT_DEBUG_INCOMING)      
      if(debugEnabled){        
        if(isprint(b) || isspace(b)) Serial.print((char) b);
        else{
          Serial.print(" 0x");
          if(b < 0x10) Serial.print('0');
          Serial.println(b&0xff, HEX);
        }
      }
#endif

      if(!writeToInputBuffer(b)){
        // TODO: complain _really_ loudly if this happens      
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)              
        Serial.println("PANIC4");
        printDebugWindow();
        numIncomingBytesPending = 0; // the transfer has failed
#endif        
        break; // out of space in application buffer                
      }

      if(numIncomingBytesPending == 0){
        // in this case we should stop processing bytes
        // we don't want to consume more bytes than were incoming in this function!
        break;
      }
    }
  }
  // else {
  //   // there are no bytes pending
  //   // ret will be false
  // }
  
  return ret; 
  // ret is true if an only if 
  //   at least one byte was put into the application data input buffer
}

// this should get called _before_ any AT communication to the ESP8266
void ESP8266_AT_Client::waitForIncomingDataToComplete(void){
  // just calls processIncomingAfterColon in a spin loop, with timeout
  // timeout after 500ms of inactivity
  uint32_t current_time = millis();    
  uint32_t previous_time = current_time;
  const int32_t timeout_interval = 500;  // signed for comparison / overflow  

  boolean dataWasIncoming = (numIncomingBytesPending > 0);
  while(numIncomingBytesPending > 0){
    current_time = millis();

    boolean gotData = processIncomingAfterColon(); // decrements numIncomingBytesPending
    if(gotData){
      previous_time = current_time;
    }
    
    if (current_time - previous_time >= timeout_interval){
      // TODO: complain _really_ loudly if this happens
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)            
      Serial.println("PANIC5");
      printDebugWindow();
#endif
      
      // probably the connection is lost
      socket_connected = false;
      numIncomingBytesPending = 0; // give up

      return; // timeout
    }
  }

#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)      
  if(dataWasIncoming && (numIncomingBytesPending > 0)){
    // TODO: complain _really_ loudly if this happens
    Serial.println("PANIC8");
    printDebugWindow();
  }
#endif

}

// this self-contained state machine should be called with _every_ incoming byte
// it returns true if the sequence IPD+, is received
// when that happens the state machine also resets / clears
// should also handle CLOSED and UNLINK receipt handling
// this is implemented using a switch statement acting as a jump table
// with the intention of it being speedy
boolean ESP8266_AT_Client::updatePlusIpdState(uint8_t chr){   
  static char lastCharAccepted = ' ';
  static uint8_t pursuitString = 0; 
  static uint8_t pursuitDepth = 0;
  
  // pursuitString = 0 is not yet in pursuit
  // pursuitString = 1 is the pursuit of '+IPD,'
  // pursuitString = 2 is the pursuit of 'CLOSED'
  // pursuitString = 3 is the pursuit of 'UNLINK'
  // pursuitString = 4 is the pursuit of 'DISCONNECT'
  // pursuitString = 5 is the pursuit of 'OK\r\n'
  // pursuitString = 6 is the pursuit of 'ERROR\r\n'
  // pursuitString = 7 is the pursuit of 'FAIL' (i.e. SEND FAIL)
  // pursuitString = 8 is the pursuit of 'ready'
  // pursuitString = 9 is the pursuit of 'GOT IP'
  // pursuitString = 10 is the pursuit of 'WIFI ' -> transfers to GOT IP or DISCONNECT
  // pursuitString = 11 is the pursuit of 'SEND' -> transfers to FAIL or OK (no \r\n required)
  // pursuitString = 12 is the pursuit of 'OK' (no \r\n required)  
  boolean ret = false;
  char c = (char) chr;

  switch(pursuitString){
  case 0: // not yet in pursuit
    ok_flag = false;
    send_ok_flag = false;
    error_flag = false;  
    ready_flag = false;        
    switch(c){
    case '+': // +IPD,
      // Serial.print("*+");
      pursuitString = 1; lastCharAccepted = c;
      break;
    case 'C': // CLOSED,
      // Serial.print("*C");
      pursuitString = 2; lastCharAccepted = c;
      break;
    case 'U': // UNLINK
      // Serial.print("*U");
      pursuitString = 3; pursuitDepth = 1; lastCharAccepted = c;
      break;   
    // case D: 4 = DISCONNECT only reachable through 'WIFI '    
    case 'O': // PURSUIT of OK
      // Serial.print("*O");    
      pursuitString = 5; lastCharAccepted = c;
      break;
    case 'E': // PURSUIT of ERROR
      // Serial.print("*E");
      pursuitString = 6; pursuitDepth = 1; lastCharAccepted = c;
      break;
    // case 'F': 7 = FAIL only reachable through 'SEND '
    case 'r': // PURSUIT of ready
      // Serial.print("*r");    
      pursuitString = 8; lastCharAccepted = c;
      break;
    // case G: 9 = GOT IP only reachable through 'WIFI '             
    case 'W': // WIFI
      // Serial.print("*W");
      pursuitString = 10; pursuitDepth = 1; lastCharAccepted = c;
      break;
    case 'S': // 'SEND '
      // Serial.print("*S");    
      pursuitString = 11; lastCharAccepted = c;
      break;       
    // case 'O': 12 = OK only reachable through 'SEND '
    }
    // intentionally leaving out default case here so we don't 
    // needlessly execute code on every byte received
    break;
  case 1: // +IPD,
    switch(lastCharAccepted){
    case '+':
      if(c == 'I') { lastCharAccepted = c; }
      else { pursuitString = 0; }
      break;
    case 'I':
      if(c == 'P') { lastCharAccepted = c; }
      else { pursuitString = 0; }
      break;
    case 'P':
      if(c == 'D') { lastCharAccepted = c; }
      else { pursuitString = 0; }
      break;
    case 'D':
      if(c == ',') {        
        // Serial.print('!');
        ret = true; // this is the only way it can be true
      }
      // unconditionally clear the state machine after ','
      pursuitString = 0;
      break;
    default: 
      pursuitString = 0;
      break;
    }
    break;
  case 2: // CLOSED
    switch(lastCharAccepted){
    case 'C':
      if(c == 'L') { lastCharAccepted = c; }
      else { pursuitString = 0; }
      break;
    case 'L':
      if(c == 'O') { lastCharAccepted = c; }
      else { pursuitString = 0; }
      break;
    case 'O':
      if(c == 'S') { lastCharAccepted = c; }
      else { pursuitString = 0; }
      break;
    case 'S':
      if(c == 'E') { lastCharAccepted = c; }
      else { pursuitString = 0; }
      break;      
    case 'E':
      if(c == 'D'){
        // Serial.print('!');
        // handle seeing CLOSED
        socket_connected = false;
        // Serial.println("socket closed because saw CLOSED");
      }
      // unconditionally clear the state machine after 'D'
      pursuitString = 0;
      break;
    default: 
      pursuitString = 0;
      break;
    }
    break;    
  case 3: // UNLINK
    switch(lastCharAccepted){
    case 'U':
      if(c == 'N') { lastCharAccepted = c; pursuitDepth++; }
      else { pursuitString = 0; }
      break;
    case 'N':
      // this is the only slightly wierd case because UNLINK has two N's
      // and it's therefore also the only reason I keep the pursuitDepth variable
      // maintaining it also doesn't matter once it increments past 2
      if(pursuitDepth == 2){ // after the first N in UNLINK
        if(c == 'L') { lastCharAccepted = c; pursuitDepth++; }
        else { pursuitString = 0; }
      }
      else if(pursuitDepth == 5){ // after the second N in UNLINK
        if(c == 'K'){
          // Serial.print('!');
          // handle seeing UNLINK
          socket_connected = false;
          // Serial.println("socket closed because saw UNLINK");
        }
        // unconditionally clear the state machine after 'K'
        pursuitString = 0;
      }
      else { pursuitString = 0; }
      break;
    case 'L':
      if(c == 'I') { lastCharAccepted = c; pursuitDepth++; }
      else { pursuitString = 0; }
      break;
    case 'I':
      if(c == 'N') { lastCharAccepted = c; pursuitDepth++; }
      else { pursuitString = 0; }
      break;      
    default: 
      pursuitString = 0;
      break;
    }
    break;
  case 4:
    switch(lastCharAccepted){ 
    case 'D':
      if(c == 'I') { lastCharAccepted = c; pursuitDepth++; }
      else { pursuitString = 0; }
      break;      
    case 'S':
      if(c == 'C') { lastCharAccepted = c; pursuitDepth++; }
      else { pursuitString = 0; }
      break;
    case 'C':                  //                           v    v
      if(pursuitDepth == 0x4){ // after the first C in 'DISCONNECT'
                               //                       0123456789
        if(c == 'O') { lastCharAccepted = c; pursuitDepth++; }
        else { pursuitString = 0; }
      }
      else if(pursuitDepth == 0x9){ // after the second C in 'DISCONNECT'
        if(c == 'T') { 
          // Serial.print('!');
          // this is a goal state
          wifi_is_connected = false;
          socket_connected = false; // that too
        }
        // unconditionally clear the state machine after 'T'
        pursuitString = 0;
      }
      else { pursuitString = 0; }
      break;                                
    case 'O':
      if(c == 'N') { lastCharAccepted = c; pursuitDepth++; }
      else { pursuitString = 0; }
      break;
    case 'N':                  //                                  vv
      if(pursuitDepth == 0xb){ // after the first N in 'WIFI DISCONNECT'
                               //                       0123456789abcde
        if(c == 'N') { lastCharAccepted = c; pursuitDepth++; }
        else { pursuitString = 0; }
      }
      else if(pursuitDepth == 0xc){ // after the second N in 'WIFI DISCONNECT'
        if(c == 'E') { lastCharAccepted = c; pursuitDepth++; }
        else { pursuitString = 0; }
      }
      else { pursuitString = 0; }
      break;          
    case 'E':
      if(c == 'C') { lastCharAccepted = c; pursuitDepth++; }
      else { pursuitString = 0; }
      break;       
    default: 
      pursuitString = 0;
      break;
    }
    break;
  case 10:
    switch(lastCharAccepted){
    case 'W':
      if(c == 'I') { lastCharAccepted = c; pursuitDepth++; }
      else { pursuitString = 0; }
      break;      
    case 'I':
                             //                         v v
      if(pursuitDepth == 2){ // after the first I in 'WIFI '
                             //                       012345
        if(c == 'F') { lastCharAccepted = c; pursuitDepth++; }
        else { pursuitString = 0; }
      }
      else if(pursuitDepth == 4){ // after the second I in 'WIFI DISCONNECT'
        if(c == ' ') { lastCharAccepted = c; pursuitDepth++; }
        else { pursuitString = 0; }
      }
      else { pursuitString = 0; }
      break;
    case 'F':
      if(c == 'I') { lastCharAccepted = c; pursuitDepth++; }
      else { pursuitString = 0; }
      break;      
    case ' ':
      if(c == 'G'){
        // Serial.print("*G");
        pursuitString = 9;
      }
      else if(c == 'D'){
        // transfer to DISCONNECT pursuit string
        // Serial.print("*D");
        pursuitString = 4; 
      }
      else{
        // transfer to DISCONNECT pursuit string
        // goal state
        // unconditionally clear the state machine after ' G' or ' D'
        pursuitString = 0;                    
      }

      // applicable to either 'G' or 'D', no-op for others because pursuitString goes back to 0
      pursuitDepth = 1;      
      lastCharAccepted = c;      
      break;   
    default: 
      pursuitString = 0;       
      break;
    }
    break;
  case 5: // OK\r\n
    switch(lastCharAccepted){
    case 'O':
      if(c == 'K'){ lastCharAccepted = c; }
      else { pursuitString = 0; }
      break;
    case 'K':
      if(c == '\r'){ lastCharAccepted = c; }
      else { pursuitString = 0; }
      break;      
    case '\r':
      if(c == '\n'){
        // Serial.print('!');
        // this is a goal state      
        ok_flag = true;        
      }

      // unconditionally clear the state machine after 'K'
      pursuitString = 0;    
      break;
    default: 
      pursuitString = 0;       
      break;
    }
    break;
  case 6: // ERROR\r\n
    switch(lastCharAccepted){
    case 'E':
      if(c == 'R') { lastCharAccepted = c;  pursuitDepth++; }
      else { pursuitString = 0; }
      break;
    case 'R':                  //                         v
      if(pursuitDepth == 0x2){ // after the first R in 'ERROR\r\n'
                               //                       01234 5 6
        if(c == 'R') { lastCharAccepted = c; pursuitDepth++; }
        else { pursuitString = 0; }
      }                             //                          v
      else if(pursuitDepth == 0x3){ // after the first R in 'ERROR\r\n'
                                    //                       01234 5 6                               
        if(c == 'O') { lastCharAccepted = c; pursuitDepth++; }
        else { pursuitString = 0; }
      }
      else { pursuitString = 0; }
    case 'O':
      if(c == 'R'){
        // Serial.print('!');
        // this is a goal sta'te     
        error_flag = true;
      }

      // unconditionally clear the state machine after third 'R'
      pursuitString = 0;    
      break;      
    default:
      pursuitString = 0;
      break;
    }           
    break;
  case 7: // FAIL
    switch(lastCharAccepted){
    case 'F':
      if(c == 'A') { lastCharAccepted = c; }
      else { pursuitString = 0; }
      break;    
    case 'A':
      if(c == 'I') { lastCharAccepted = c; }
      else { pursuitString = 0; }
      break;      
    case 'I':
      if(c == 'L') { 
        // Serial.print('!');
        // this is a goal state     
        error_flag = true;
      }
      // unconditionally clear the state machine after 'L'
      pursuitString = 0;
      break;            
    default: 
      pursuitString = 0;
      break; 
    }   
    break; 
  case 8: // ready
    switch(lastCharAccepted){
    case 'r':
      if(c == 'e') { lastCharAccepted = c; }
      else { pursuitString = 0; }
      break;    
    case 'e':
      if(c == 'a') { lastCharAccepted = c; }
      else { pursuitString = 0; }
      break; 
    case 'a':
      if(c == 'd') { lastCharAccepted = c; }
      else { pursuitString = 0; }
      break;            
    case 'd':
      if(c == 'y'){
        // Serial.print('!');
        // this is a goal state
        ready_flag = true;    
      }

      // unconditionally clear the state machine after 'y'
      pursuitString = 0;    
      break;
    default:
      pursuitString = 0;
      break;
    }     
    break;   
  case 9: // GOT IP,
    switch(lastCharAccepted){
    case 'G':
      if(c == 'O') { lastCharAccepted = c; }
      else { pursuitString = 0; }
      break;
    case 'O':
      if(c == 'T') { lastCharAccepted = c; }
      else { pursuitString = 0; }
      break;
    case 'T':
      if(c == ' ') { lastCharAccepted = c; }
      else { pursuitString = 0; }
      break;
    case ' ':
      if(c == 'I') { lastCharAccepted = c; }
      else { pursuitString = 0; }
      break;      
    case 'I':
      if(c == 'P') {   
        // goal state            
        // Serial.print('!');
        wifi_is_connected = true;
      }
      // unconditionally clear the state machine after 'P'
      pursuitString = 0;
      break;
    default: 
      pursuitString = 0;
      break;
    }
    break; 
  case 11: // 'SEND '
    switch(lastCharAccepted){
    case 'S':
      if(c == 'E') { lastCharAccepted = c; }
      else { pursuitString = 0; }
      break;      
    case 'E':
      if(c == 'N') { lastCharAccepted = c; }
      else { pursuitString = 0; }
      break;
    case 'N':
      if(c == 'D') { lastCharAccepted = c; }
      else { pursuitString = 0; }
      break;      
    case 'D':
      if(c == ' ') { lastCharAccepted = c; }
      else { pursuitString = 0; }
      break;            
    case ' ':
      if(c == 'O'){
        // transfer to OK pursuit string
        // Serial.print("*O");
        pursuitString = 12;
      }
      else if(c == 'F'){
        // transfer to FAIL pursuit string
        // Serial.print("*F");
        pursuitString = 7; 
      }
      else{      
        // goal state
        // unconditionally clear the state machine after ' O' or ' F'
        pursuitString = 0;                    
      }

      // applicable to either 'O' or 'F', no-op for others because pursuitString goes back to 0
      lastCharAccepted = c;  
      break;   
    default: 
      pursuitString = 0;       
      break;
    } 
    break;   
  case 12: // 'OK' 
    switch(lastCharAccepted){
    case 'O':
      if(c == 'K') { 
        // goal state            
        // Serial.print('!'); 
        send_ok_flag = true;       
      }
      // unconditionally clear the state machine after 'K'
      pursuitString = 0;            
      break;
    default: 
      pursuitString = 0;       
      break;
    }
    break;      
  default:
    pursuitString = 0;
    break;
  }
  
  return ret;
}
