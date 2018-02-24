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
#define DEBUG_WINDOW_SIZE (50)

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
  for(;;); // die
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
  this->listener_started = false;
  this->input_buffer = NULL;
  this->input_buffer_length = 0;
  this->input_buffer_read_ptr = NULL;
  this->input_buffer_write_ptr = NULL;
  this->input_buffer_tail_ptr = NULL;
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
  this->listener_started = false;
  this->input_buffer = NULL;
  this->input_buffer_length = 0;
  this->input_buffer_read_ptr = NULL;
  this->input_buffer_write_ptr = NULL;
  this->input_buffer_tail_ptr = NULL;
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
  this->listener_started = false;
  this->input_buffer = buf;
  this->input_buffer_length = buf_length;
  this->input_buffer_read_ptr = buf;
  this->input_buffer_write_ptr = buf;
  this->input_buffer_tail_ptr = &(buf[buf_length-1]);
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
  input_buffer_read_ptr = input_buffer;
  input_buffer_write_ptr = input_buffer;
  input_buffer_tail_ptr = &(input_buffer[input_buffer_length-1]);
  num_consumed_bytes_in_input_buffer = 0;
  num_free_bytes_in_input_buffer = input_buffer_length;
  numIncomingBytesPending = 0;

  socket_connected = false;
  wifi_is_connected = false;
  listener_started = false;

  pinMode(enable_pin, OUTPUT);
  digitalWrite(enable_pin, LOW);
  delay(50);
  digitalWrite(enable_pin, HIGH);

  ESP8266_DEBUG("ESP8266 Hello World.");

  if(readStreamUntil("ready", 10000)){
    ESP8266_DEBUG("Received 'ready'");
  }
  else{

#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
    Serial.println("PANIC6");
    printDebugWindow();
#endif

    return false;
  }

  return true;
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
  input_buffer_read_ptr = buf;
  input_buffer_write_ptr = buf;
  input_buffer_tail_ptr = &(buf[buf_length-1]);
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

  int ret = 2; // initialize to error
  socket_connected = false;
  listener_started = false;

  ESP8266_DEBUG("Connecting to ", (char *) host);

  waitForIncomingDataToComplete();
  flushInput();
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

  // ESP8266 responds with either "OK", "ERROR", or "ALREADY CONNECT"
  clearTargetMatchArray();
  addStringToTargetMatchList("OK");
  addStringToTargetMatchList("ERROR");
  addStringToTargetMatchList("ALREADY CONNECT");
  uint8_t match_index = 0xFF;
  if(readStreamUntil(&match_index)){
     if(match_index == 0){ // got "OK"
       ESP8266_DEBUG("Connected");
       ret = 1; // success
     }
     else if(match_index == 2){
       ESP8266_DEBUG("Already Connected");
       ret = 1; // success
     }
     else{
       ESP8266_DEBUG("Failed");
       ret = 2; // error
     }
  }

  if(ret == 1){
    socket_connected = true;
    socket_type = proto;
  }

  return ret;
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

  if(strlen(mac_str) == 17){ // e.g. 00:04:4a:23:11:7b
    waitForIncomingDataToComplete();    
    flushInput();
    streamWrite("AT+CIPSTAMAC_CUR=\"");
    streamWrite(mac_str);
    streamWrite("\"\r\n");

    clearTargetMatchArray();
    addStringToTargetMatchList("OK");
    addStringToTargetMatchList("WIFI DISCONNECT");
    addStringToTargetMatchList("WIFI CONNECTED");
    addStringToTargetMatchList("WIFI GOT IP");
    uint8_t match_index = 0xFF;    
    while(readStreamUntil(&match_index, 5000)){
      if(match_index == 0){
        ret = true;
        break;
      }

      if(match_index == 2){
        wifi_is_connected = true;
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
  streamWrite("AT+CIPMUX=1");
  streamWrite("\r\n");

  clearTargetMatchArray();
  addStringToTargetMatchList("OK");

  uint8_t match_index = 0xFF;
  while(readStreamUntil(&match_index, 1000)){
    if(match_index == 0){
      ret = true;
      break;
    }
    else if(match_index != 0xFF){
      break;
    }
  }

  if(ret){ // so far so good

    delay(100);
    flushInput();
    ret = false;

    waitForIncomingDataToComplete();
    streamWrite("AT+CIPSERVER=1,");
    streamWrite((uint32_t) port);
    streamWrite("\r\n");

    clearTargetMatchArray();
    addStringToTargetMatchList("OK");
    addStringToTargetMatchList("ERROR");

    match_index = 0xFF;
    while(readStreamUntil(&match_index, 100)){
      if(match_index == 0){
        listener_started = true;
        ret = true;
        break;
      }
      else if(match_index != 0xFF){
        break;
      }
    }
  }

  return ret;
}

boolean ESP8266_AT_Client::configureSoftAP(const char *ssid, const char *pwd, uint8_t channel, uint8_t sec){

  boolean ret = false;
  
  waitForIncomingDataToComplete();
  flushInput();
  streamWrite("AT+CWSAP_CUR=\"");
  streamWrite(ssid);
  streamWrite("\",\"");
  streamWrite(pwd);
  streamWrite("\",");
  streamWrite((uint32_t) channel);
  streamWrite(",");
  streamWrite((uint32_t) sec);
  streamWrite("\r\n");

  clearTargetMatchArray();
  addStringToTargetMatchList("OK");
  addStringToTargetMatchList("ERROR");
  uint8_t match_index = 0xFF;
  while(readStreamUntil(&match_index, 10000)){
    if(match_index == 0){
      ret = true;
      break;
    }
    else if(match_index != 0xFF){
      break;
    }
  }

  return ret;
}

// 0 : disable sleep mode
// 1 : light-sleep mode
// 2 : modem-sleep mode
boolean ESP8266_AT_Client::sleep(uint8_t mode){

  boolean ret = false;

  waitForIncomingDataToComplete();
  streamWrite("AT+SLEEP=");
  streamWrite((uint32_t) mode);
  streamWrite("\r\n");

  if(readStreamUntil("OK", 100)){
    ret = true;
  }

  return ret;
}

// note dnsServer is currently ignored as there is no direct support for it in the AT command set, afaict
// at any rate, we are currently *emulating* DNS in this library, not actually sending explicit DNS requests to a name server
boolean ESP8266_AT_Client::setStaticIPAddress(uint32_t ipAddress, uint32_t netMask, uint32_t defaultGateway, uint32_t dnsServer){
  boolean ret = false;
  char ip_str[16] = {0};
  char netMask_str[16] = {0};
  char defaultGateway_str[16] = {0};
  char dnsServer_str[16] = {0};

  IpUint32ToString(ipAddress, (char *) ip_str);
  IpUint32ToString(netMask, (char *) netMask_str);
  IpUint32ToString(defaultGateway, (char *) defaultGateway_str);
  IpUint32ToString(dnsServer, (char *) dnsServer_str);

  waitForIncomingDataToComplete();
  streamWrite("AT+CIPSTA_CUR=\"");
  streamWrite((char *) ip_str);
  streamWrite("\",\"");
  streamWrite((char *) defaultGateway_str);
  streamWrite("\",\"");
  streamWrite((char *) netMask_str);
  streamWrite("\"\r\n");

  if(readStreamUntil("OK", 1000)){
    ret = true;
  }

  return ret;
}

boolean ESP8266_AT_Client::setDHCP(void){
  boolean ret = false;

  waitForIncomingDataToComplete();
  streamWrite("AT+CWDHCP_CUR=1,1\r\n");

  clearTargetMatchArray();
  addStringToTargetMatchList("OK");
  uint8_t match_index = 0xff;
  while(readStreamUntil(&match_index, 2000)){
    if(match_index == 0){
      ret = true;
      break;
    }
  }

  return ret;
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

  // expect to get ">"
  // then send the bytes
  // then expect to get SEND OK

  clearTargetMatchArray();
  addStringToTargetMatchList(">");
  addStringToTargetMatchList("SEND OK");

  waitForIncomingDataToComplete();
  streamWrite("AT+CIPSEND=");
  if(listener_started){
    // TODO: this assumes the link id is zero, and so only supports one connection
    streamWrite("0,");
  }
  streamWrite((uint32_t) sz);
  streamWrite("\r\n");

  boolean timeout = false;
  boolean complete = false;
  uint8_t match_index = 0xFF;
  while(!timeout && !complete){
    readStreamUntil(&match_index, 500);
    switch(match_index){
    case 0:
      ret = streamWrite(buf, sz); // pass it along
      break;
    case 1: 
      complete = true;
      break;
    default: //timeout
      timeout = true;            
      break;
    }
    match_index = 0xFF;
  }

  return ret;
}

/** Check if there is data pending receipt
	@return 1 if exists, 0 if not exists
 */
int ESP8266_AT_Client::available(){
  // an intent to read() is always preceded by a call to available(),
  // if the caller knows what is good for them,
  // so this is where we need to perform asynchronous receipt of +IPD data
  uint16_t bytesAvailable = stream->available();
  if(bytesAvailable > ESP8266_AT_Client::bytesAvailableMax){
    ESP8266_AT_Client::bytesAvailableMax = bytesAvailable;
  }

#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)            
  if(bytesAvailable > 50){
    // TODO: complain loudly if this happens 
    Serial.println("PANIC13");      
    printDebugWindow();
  }        
#endif    

  if(bytesAvailable > 0){
    while(stream->available()){    
      streamReadChar();
    }
  }

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
	@returns *input_buffer_read_ptr
 */
int ESP8266_AT_Client::peek(){
  return *input_buffer_read_ptr;
}

/** Flush response buffer
 */
void ESP8266_AT_Client::flush(){
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
    streamWrite("AT+CIPCLOSE");
    if(listener_started){
      streamWrite("=0"); //TODO: assumes target is link id 0
    }
    streamWrite("\r\n");

    // ESP8266 responds with either "OK", "ERROR"
    clearTargetMatchArray();
    addStringToTargetMatchList("CLOSED");
    addStringToTargetMatchList("ERROR");

    uint8_t match_index = 0xFF;
    if(readStreamUntil(&match_index, 5000)){
       if(match_index == 0){ // got "CLOSED"
         ESP8266_DEBUG("Close Succeeded");
       }
       else{
         ESP8266_DEBUG("Close Failed");
       }
    }

  }  

  // and drop all the remaining unread user buffer data
  while(num_consumed_bytes_in_input_buffer > 0){
    readFromInputBuffer();
  }

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
  clearTargetMatchArray();
  addStringToTargetMatchList("+CWLAP:(");
  addStringToTargetMatchList(")\r\n");
  addStringToTargetMatchList("OK\r\n");

  waitForIncomingDataToComplete();
  streamWrite("AT+CWLAP");
  streamWrite("\r\n");

  uint8_t match_index = 0xFF;
  char line[128] = {0};
  uint8_t result_number = 0;

  while(readStreamUntil((uint8_t *) &match_index, &(line[0]), 128, timeout_ms)){
    if(match_index == 1){ // got )
      ap_scan_result_t res = {0};
      parseScanResult(&res, line);
      memset((char *) line, 0, 128);
      result_number++;
      if((strcmp(&(res.ssid[0]), ssid) == 0) && (res.rssi > max_rssi)){
        *result = res;
        max_rssi = res.rssi;
        ret = true;
      }
    }
    else if(match_index == 2){
      break;
    }
  }

  *num_results_found = result_number;

  return ret;
}

// returns true if *any* SSIDs are found
boolean ESP8266_AT_Client::scanAccessPoints(ap_scan_result_t * results, uint8_t max_num_results, uint8_t * num_results_found, uint32_t timeout_ms){
  boolean ret = false;

  clearTargetMatchArray();
  addStringToTargetMatchList("+CWLAP:(");
  addStringToTargetMatchList(")\r\n");
  addStringToTargetMatchList("OK\r\n");

  waitForIncomingDataToComplete();
  streamWrite("AT+CWLAP");
  streamWrite("\r\n");

  uint8_t match_index = 0xFF;
  char line[128] = {0};
  uint8_t result_number = 0;

  while(readStreamUntil((uint8_t *) &match_index, &(line[0]), 128, timeout_ms)){
    ret = true;

    if(match_index == 1){ // got )
      if(result_number < max_num_results){
        parseScanResult(&(results[result_number]), line);
        memset((char *) line, 0, 128);
      }
      result_number++;
    }
    else if(match_index == 2){
      break;
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

  clearTargetMatchArray();
  addStringToTargetMatchList("STATUS:2");   // got ip
  addStringToTargetMatchList("STATUS:3");   // connected
  addStringToTargetMatchList("STATUS:4");   // disconnected
  addStringToTargetMatchList("OK\r\n");

  waitForIncomingDataToComplete();
  streamWrite("AT+CIPSTATUS");
  streamWrite("\r\n");

  uint8_t match_index = 0xFF;
  if(readStreamUntil(&match_index, 100)){
    if((match_index == 0) || (match_index == 1) || ((match_index == 2) && (socket_type == ESP8266_UDP))){
      if(readStreamUntil("+CIPSTATUS:", 100)){
        // we'll see three quotation marks before we reach the Remote IP address
        if(readStreamUntil("\"", 100) && readStreamUntil("\"", 100) && readStreamUntil("\"", 100)){
          char remote_ip_str[16] = {0};
          if(readStreamUntil("\"", &(remote_ip_str[0]), 16, 100)){
            ret = stringToIpUint32((char *) remote_ip_str, ip);
            readStreamUntil("OK\r\n", 100);
          }
        }
      }
    }
  }

  return ret;
}

uint8_t ESP8266_AT_Client::connected(boolean actively_check){
  uint8_t ret = 1; // assume we are connected
  uint8_t called_receive = 0;

  if(actively_check){ // things seem go bad if you do this in a tight loop
    // set up an AT command and send it
    // then return whether or not it succeeded
    
    clearTargetMatchArray();
    addStringToTargetMatchList("STATUS:2"); // got ip
    addStringToTargetMatchList("STATUS:3"); // connected
    addStringToTargetMatchList("STATUS:4"); // disconnected
    addStringToTargetMatchList("OK\r\n");

    // you have to get "STATUS:" and a numeric code
    // then you have *may* get "+CIPSTATUS:"
    // then you get "OK"
    waitForIncomingDataToComplete();
    flushInput();
    streamWrite("AT+CIPSTATUS");
    streamWrite("\r\n");

    uint8_t match_index = 0xFF;
    if(readStreamUntil(&match_index, 100)){
      switch(match_index){
      case 2: //disconnected
        if(socket_type == ESP8266_TCP){
          socket_connected = false;
          ret = 0;
        }
        break;
      case 0:
      case 1:
      case 3:
      default:
          // nothing to do
        break;
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
  boolean ret = false;

  clearTargetMatchArray();
  addStringToTargetMatchList("+CIPSTA:ip:\"");
  addStringToTargetMatchList("+CIPSTA:gateway:\"");
  addStringToTargetMatchList("+CIPSTA:netmask:\"");
  addStringToTargetMatchList("\""); // this is tricky, since it's a subset of other match strings, it must come after them
  addStringToTargetMatchList("OK\r\n");

  waitForIncomingDataToComplete();
  flushInput();
  streamWrite("AT+CIPSTA?");
  streamWrite("\r\n");

  uint8_t match_index = 0xFF;
  char tmp[32] = {0};
  uint8_t which_ip = 0xFF; // 0 for ip, 1 for gateway, 2 for netmask
  while(readStreamUntil(&match_index, &(tmp[0]), 32, 100)){
    ret = true;
    if((which_ip < 3) && (match_index == 3) && (strlen(tmp) <= 15)){
      switch(which_ip){
      case 0:
        strcpy(ip_str, tmp);
        break;
      case 1:
        strcpy(gateway_str, tmp);
        break;
      case 2:
        strcpy(netmask_str, tmp);
        break;
      default: break;
      }
    }

    which_ip = 0xFF;
    if(match_index < 3){
      which_ip = match_index;
      memset(tmp, 0, 32);
    }

    if(match_index == 4){
      break;
    }
  }

  return ret;
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
  boolean ret = false;

  clearTargetMatchArray();
  addStringToTargetMatchList("+CIFSR:STAMAC,\""); // connected
  addStringToTargetMatchList("OK\r\n");
  addStringToTargetMatchList("ERROR\r\n");

  waitForIncomingDataToComplete();
  flushInput();
  streamWrite("AT+CIFSR");
  streamWrite("\r\n");

  uint8_t match_index = 0xFF;
  if(readStreamUntil(&match_index, 100)){
    if(match_index == 0){
      char tmp[32] = {0};
      if(readStreamUntil("\"", &(tmp[0]), 32, 10)){
        strncpy(mac_str, tmp, 18); // an mac address is at most 17 characters
        ret = true;
      }

      readStreamUntil("OK", 100); // clear out the OK
    }
  }
  else{
    // Timeout
  }

  return ret;
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
  streamWrite("AT+CIPSTART=\"UDP\",\"");
  streamWrite(hostname);
  streamWrite("\",7\r\n");

  if(readStreamUntil("OK", timeout_ms)){
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


/** Adds str to list safely
	@param list
	@param str
	@param max_num_entries
	@return returns true if the item is successfully added and false otherwise
 */
boolean ESP8266_AT_Client::addStringToTargetMatchList(char * str){
  uint16_t free_index = 0xFFFF;

  if(strlen(str) <= ESP8266_AT_CLIENT_MAX_STRING_LENGTH){

    // search the list for an empty space
    // the last empty space must remain empty
    // so don't include it in the search
    for(uint16_t ii = 0; ii < ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES; ii++){
      uint16_t len = strlen(&(target_match_array[ii][0]));
      if(0 == len){
        free_index = ii;
        break;
      }
    }

    // free index is the first empty space in the list
    // or 0xFFFF if no free entries were found

    // if free index points to a viable position
    // then copy the candidate string into that position
    // and limit the number of characters copied
    if(free_index < ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES){
      char * tgt_addr = &(target_match_array[free_index][0]);
      strncpy(tgt_addr, str, ESP8266_AT_CLIENT_MAX_STRING_LENGTH + 1);  // copy the string in
      target_match_lengths[free_index] = strlen(tgt_addr);
      return true;
    }

  }

  return false;
}

// pass in an array of strings to match against
// the list is presumed to be terminated by a NULL string
// this function can only handle up to ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES matching targets
// TODO: this function should get another argument to reset its internal state
//       because the caller decides it failed irrecoverably for some reason it might never recover?
boolean ESP8266_AT_Client::readStreamUntil(uint8_t * match_idx, char * target_buffer, uint16_t target_buffer_length, int32_t timeout_ms, boolean reset_timeout_on_possible_rx){
  boolean match_found = false;
  static boolean initial_call = true;
  static uint16_t local_target_buffer_index = 0;
  static uint8_t match_char_idx[ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES] = {0};
  unsigned long previousMillis = millis();
  boolean first_match_character_received = false;

#ifdef ESP8266_AT_CLIENT_ENABLE_DEBUG
  if((debugStream != NULL) && debugEnabled){
    ESP8266_DEBUG("+++");
    for(uint8_t ii = 0; ii < ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES; ii++){
      uint16_t target_match_length = target_match_lengths[ii];
      if(target_match_length == 0){
        break;
      }
      ESP8266_DEBUG("  Waiting for ", target_match_array[ii]);
    }
    ESP8266_DEBUG("===");
  }
#endif

  if(initial_call){
    initial_call = false;
    //debugStream->println("\nbegin>");
  }

  while(!match_found){ // until a match is found
    unsigned long currentMillis = millis();
    if((timeout_ms > 0) && (currentMillis - previousMillis >= timeout_ms)){
       break;
    }

    uint8_t bytesAvailable = stream->available();    
    if(bytesAvailable > ESP8266_AT_Client::bytesAvailableMax){
      ESP8266_AT_Client::bytesAvailableMax = bytesAvailable;
    }    

#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)        
    if(bytesAvailable > 50){
      // TODO: complain loudly if this happens 
      Serial.println("PANIC11");      
      printDebugWindow();
    }
#endif

    if(bytesAvailable > 0){

      // NOTE: not sure we need to do this but this seems a better place for it than on _any_ character rx
      // the problem with that is that the timeout can be totally disregarded in softAP mode when a
      // client is polling and we are waiting on a long timeout (e.g. 30 seconds for network connect)
      if(reset_timeout_on_possible_rx){
        previousMillis = millis(); // reset the timeout on any sequence-matching character received
      }

      int chr = streamReadChar(); // read a character      
      if(chr == -1){
        // reset the timeout if this happens
        previousMillis = millis(); 
        continue;
      }
      
#ifdef ESP8266_AT_CLIENT_DEBUG_ECHO_EVERYTHING
      if(debugStream != NULL && debugEnabled) debugStream->print(chr); // echo the received characters to the Serial Monitor
#endif
      // if(debugEnabled) Serial.print(chr); // a less complicated way to echo everything the ESP says

      // for each target match
      for(uint8_t ii = 0; ii < ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES; ii++){
        uint16_t target_match_length = target_match_lengths[ii];
        // an empty string in the list signals the end of the list
        if(target_match_length == 0){
          break;
        }

        // if the current character is a match for this string
        // advance it's match index,
        // otherwise reset its match index
        if(chr == target_match_array[ii][match_char_idx[ii]]){
           match_char_idx[ii]++;
        }
        else{
           match_char_idx[ii] = 0;
        }

        // if the match index is equal to the length of the string
        // then it's a complete match
        // return the string index that matched into match_idx
        // and return true to the caller
        if(match_char_idx[ii] >= target_match_length){
          *match_idx = ii;
          match_found = true;

          // copy the collected local data into the caller's buffer
          if(target_buffer != NULL){
            if(local_target_buffer_index > target_buffer_length){
              ESP8266_DEBUG("Warn: caller's buffer is smaller than needed to contain", target_buffer);
            }
          }

          // reset the stateful variables
          memset(match_char_idx, 0, ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES);
          local_target_buffer_index = 0;
          initial_call = true;
          //debugStream->println("<end");
          break;
        }
      }

      if(!match_found && target_buffer != NULL){
        if(local_target_buffer_index < target_buffer_length - 1){
          target_buffer[local_target_buffer_index++] = chr;
        }
        else{
          ESP8266_DEBUG("Target buffer would overflow");
          break; // target buffer overflow
        }
      }
    }
  }

  ESP8266_DEBUG("*** ", (uint8_t) match_found);
  ESP8266_DEBUG("==> ", (uint8_t) *match_idx);
  // delayMicroseconds(10); // i'm not sure why, but it's more well behaved with this delay injected
                         // one theory is that it gives the Serial output time to clear?

#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)            
  if(!match_found){
    Serial.println("PANIC7");
    Serial.println("+++");
    for(uint8_t ii = 0; ii < ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES; ii++){
      uint16_t target_match_length = target_match_lengths[ii];
      if(target_match_length == 0){
        break;
      }
      Serial.print("  Waiting for ");
      Serial.println(target_match_array[ii]);
    }
    Serial.println("===");   
    printDebugWindow(); 
  }
#endif  
                         
  return match_found;
}

// pass a single string to match against
// the string must not be longer than 31 characters
boolean ESP8266_AT_Client::readStreamUntil(char * target_match, char * target_buffer, uint16_t target_buffer_length, int32_t timeout_ms, boolean reset_timeout_on_possible_rx){
  uint8_t dummy_return;

  if(strlen(target_match) > 31){
    return false;
  }
  else{
    clearTargetMatchArray();
    addStringToTargetMatchList(target_match);
    return readStreamUntil(&dummy_return, target_buffer, target_buffer_length, timeout_ms, reset_timeout_on_possible_rx);
  }
}

boolean ESP8266_AT_Client::readStreamUntil(char * target_match, char * target_buffer, uint16_t target_buffer_length, int32_t timeout_ms){
  return readStreamUntil(target_match, target_buffer, target_buffer_length, timeout_ms, true); // reset timeout on possible rx
}

boolean ESP8266_AT_Client::readStreamUntil(char * target_match, int32_t timeout_ms, boolean reset_timeout_on_possible_rx){
  return readStreamUntil(target_match, NULL, 0, timeout_ms, reset_timeout_on_possible_rx);
}

boolean ESP8266_AT_Client::readStreamUntil(char * target_match, int32_t timeout_ms){
  return readStreamUntil(target_match, NULL, 0, timeout_ms);
}

boolean ESP8266_AT_Client::readStreamUntil(char * target_match){
  return readStreamUntil(target_match, -1);
}

boolean ESP8266_AT_Client::readStreamUntil(uint8_t * match_idx, char * target_buffer, uint16_t target_buffer_length, int32_t timeout_ms){
    return readStreamUntil(match_idx, target_buffer, target_buffer_length, timeout_ms, true); // reset timeout on possible rx
}

boolean ESP8266_AT_Client::readStreamUntil(uint8_t * match_idx, int32_t timeout_ms){
  return readStreamUntil(match_idx, NULL, 0, timeout_ms);
}

boolean ESP8266_AT_Client::readStreamUntil(uint8_t * match_idx){
  return readStreamUntil(match_idx, -1);
}

// writes c to the write pointer in the input buffer
// otherwise the write pointer is advanced
boolean ESP8266_AT_Client::writeToInputBuffer(uint8_t c){
  if(num_free_bytes_in_input_buffer > 0){
    *input_buffer_write_ptr = c; // write the value to the buffer
    // update the write pointer to the next location to write to within the buffer
    if(input_buffer_write_ptr == input_buffer_tail_ptr){
      input_buffer_write_ptr = input_buffer;
    }
    else{
      input_buffer_write_ptr++;
    }

    num_consumed_bytes_in_input_buffer++;
    num_free_bytes_in_input_buffer--;

    //debugStream->write(c);

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
uint8_t ESP8266_AT_Client::readFromInputBuffer(void){
  if(num_consumed_bytes_in_input_buffer > 0){
    uint8_t ret = *input_buffer_read_ptr;

    if(input_buffer_read_ptr == input_buffer_tail_ptr){
      input_buffer_read_ptr = input_buffer;
    }
    else{
      input_buffer_read_ptr++;
    }

    num_consumed_bytes_in_input_buffer--;
    num_free_bytes_in_input_buffer++;

    //debugStream->write(ret);

    return ret;
  }

  return -1;
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
      int chr = streamReadChar();
      if(chr == -1){
        continue;
      }

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
  streamWrite("AT+CWMODE_CUR=");
  streamWrite((uint32_t) mode);
  streamWrite("\r\n");

  // ESP8266 responds with either "OK", "ERROR"
  clearTargetMatchArray();
  addStringToTargetMatchList("OK");
  addStringToTargetMatchList("ERROR");

  uint8_t match_index = 0xFF;
  if(readStreamUntil(&match_index)){
     if(match_index == 0){
       ESP8266_DEBUG("Debug: NetworkMode Succeeded");
       ret = true;
     }
     else{
       ESP8266_DEBUG("Debug: NetworkMode Failed");
       ret = false;
     }
  }

  flushInput();
  return ret;
}

boolean ESP8266_AT_Client::connectToNetwork(char * ssid, char * pwd, int32_t timeout_ms, void (*onConnect)(void), boolean permanent){
  
  waitForIncomingDataToComplete();
  flushInput();
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

  // wait for connected status
  if(readStreamUntil("WIFI CONNECTED", timeout_ms, false)){  
    ESP8266_DEBUG("Connected to Network");
    if(onConnect != NULL){
      onConnect();
    }
    // wait for got IP status
    if(readStreamUntil("WIFI GOT IP", timeout_ms, false)){
       ESP8266_DEBUG("Got IP");

       if(readStreamUntil("OK")){
         wifi_is_connected = true;
         return true;
       }
    }
    else{
       ESP8266_DEBUG("Failed to get IP address");
       wifi_is_connected = false;
       return false;
    }
  }
  else{
     ESP8266_DEBUG("Failed to connect to Network");
     wifi_is_connected = false;
     return false;
  }

  wifi_is_connected = false;
  return false;
}

//TODO: Implement
boolean ESP8266_AT_Client::disconnectFromNetwork(){
  boolean ret = false;

  waitForIncomingDataToComplete();
  flushInput();
  streamWrite("AT+CWQAP");
  streamWrite("\r\n");

  if(readStreamUntil("WIFI DISCONNECT", 1000)){
     ESP8266_DEBUG("Disconnected from Network");
     ret = true;
  }
  else{
     ESP8266_DEBUG("Failed to disconnect from Network");
  }

  return ret;
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

void ESP8266_AT_Client::clearTargetMatchArray(void){
  for(uint8_t ii = 0; ii < ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES; ii++){
    target_match_array[ii][0] = NULL;
    target_match_lengths[ii] = 0;
  }
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
  boolean ret = false;

  waitForIncomingDataToComplete();
  flushInput();
  streamWrite("AT+CIUPDATE");
  streamWrite("\r\n");

  clearTargetMatchArray();
  addStringToTargetMatchList("+CIPUPDATE:1"); // 0
  addStringToTargetMatchList("ERROR");        // 1
  addStringToTargetMatchList("OK");           // 2
  addStringToTargetMatchList("ready");        // 3

  uint8_t match_index = 0xFF;
  while(readStreamUntil(&match_index, 5000)){
    if(match_index == 0){
      ret = true;
      break;
    }
    else if(match_index == 1){
      break;
    }
  }

  return ret;
}

// this function returns true as long as you don't get an ERROR response
// it ASSUMES you will call firmwareUpdateBegin and then call this function
// until it writes a STATUS of 2 (OK) into *status or it returns false (ERROR)
// see example invocation above
boolean ESP8266_AT_Client::firmwareUpdateStatus(uint8_t * status){
  boolean ret = true;

  uint8_t match_index = 0xFF;
  while(readStreamUntil(&match_index, 5000)){
    if(match_index == 1){ // ERROR index from firmwareUpdateBegin
      ret = false;
    }
    break;
  }

  *status = match_index; // so the caller knows
  return ret;
}

boolean ESP8266_AT_Client::getVersion(char * version){
  boolean ret = false;

  clearTargetMatchArray();
  addStringToTargetMatchList("AT version:"); // connected
  addStringToTargetMatchList("OK\r\n");
  addStringToTargetMatchList("ERROR\r\n");

  waitForIncomingDataToComplete();
  flushInput();
  streamWrite("AT+GMR");
  streamWrite("\r\n");

  uint8_t match_index = 0xFF;
  if(readStreamUntil(&match_index, 100)){
    if(match_index == 0){
      char tmp[16] = {0};
      if(readStreamUntil("(", &(tmp[0]), 16, 10)){
        strncpy(version, tmp, 15); // an mac address is at most 17 characters
        ret = true;
      }

      readStreamUntil("OK", 100); // clear out the OK
    }
  }
  else{
    // Timeout
  }

  return ret;
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
  streamWrite("AT+RESTORE");
  streamWrite("\r\n");
  return readStreamUntil("ready", 2000);
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

      if((bytes_written % 32) == 0){
        delay(1); // maybe don't flood the ESP8266 with bytes?
      }
    }    
  }  
  return bytes_written;
}

int16_t ESP8266_AT_Client::streamReadChar(void){
  if(numIncomingBytesPending > 0){
    processIncomingAfterColon();
    return -1;
  }
  else {
    uint8_t b = stream->read() & 0xff;
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
      return -1; // -1 is distinguishable from consumable data to the caller
                // because return type is artificially int16_t (not uint16_t)
    }
    else{
      return b;
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
      bytesAvailable--;
      previous_time = current_time;
      unsigned char b = stream->read() & 0xff;
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
  char * temp;
  numIncomingBytesPending = strtoul(&num_bytes_str[0], &temp, 10);
  if (*temp != '\0'){
    // TODO: complain loudly if this happens
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)               
    Serial.println("PANIC3");
    Serial.print("num_bytes_str was \"");
    Serial.print(num_bytes_str);
    Serial.println("\"");
    for(uint16_t ii = 0; ii < strlen(num_bytes_str); ii++){
      uint8_t b = num_bytes_str[ii];
      if(isprint(b) || isspace(b)){
        Serial.print((char) b);
      }
      else{
        Serial.print(" 0x");
        if(b < 0x10) Serial.print('0');
        Serial.print(b, HEX);
      }
    }
    printDebugWindow();
#endif    
    numIncomingBytesPending = 0;
    return; // failed to parse length
  }
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
      bytesAvailable--;

      uint8_t b = stream->read() & 0xff;
      addToDebugReadWindow(b);

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

      if(writeToInputBuffer(b)){
        ret = true;
        numIncomingBytesPending--;
      }
      else {
        // TODO: complain _really_ loudly if this happens      
#if defined(ESP8266_AT_CLIENT_ENABLE_PANIC_MESSAGES)              
        Serial.println("PANIC4");
        printDebugWindow();
        numIncomingBytesPending = 0; // the transfer has failed
#endif        
        return false; // out of space in application buffer                
      }

      if(numIncomingBytesPending == 0){
        // in this case we should stop processing bytes
        // we don't want to consume more bytes than were incoming in this function!
        break;
      }
    }
  }

  return ret;
}

// this should get called _before_ any AT communication to the ESP8266
void ESP8266_AT_Client::waitForIncomingDataToComplete(void){
  // just calls processIncomingAfterColon in a spin loop, with timeout
  // timeout after 500ms of inactivity
  uint32_t current_time = millis();    
  uint32_t previous_time = current_time;
  const int32_t timeout_interval = 500;  // signed for comparison / overflow  
  boolean dataWasIncoming = numIncomingBytesPending > 0;
  while(numIncomingBytesPending > 0){
    current_time = millis();
    boolean gotData = processIncomingAfterColon(); // decrements numIncomingBytesPending
    if(gotData){
      previous_time = current_time;
    }
    else if (current_time - previous_time >= timeout_interval){
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
  // pursuitString = 4 is the pursuit of 'WIFI DISCONNECT'
  boolean ret = false;
  char c = (char) chr;

  switch(pursuitString){
  case 0: // not yet in pursuit
    switch(c){
    case '+': // +IPD,
      pursuitString = 1; lastCharAccepted = c;
      break;
    case 'C': // CLOSED,
      pursuitString = 2; lastCharAccepted = c;
      break;
    case 'U': // UNLINK
      pursuitString = 3; pursuitDepth = 1; lastCharAccepted = c;
      break;   
    case 'W': // WIFI DISCONNECT
      pursuitString = 4; pursuitDepth = 1; lastCharAccepted = c;
      break;
    }
    // intentionally leaving out default case here so we don't 
    // needlessly execute code on every byte received
    break;
  case 1: // +IPD,
    switch(lastCharAccepted){
    case '+':
      if(c == 'I') { lastCharAccepted = c; }
      else { pursuitString = 0; lastCharAccepted = ' '; }
      break;
    case 'I':
      if(c == 'P') { lastCharAccepted = c; }
      else { pursuitString = 0; lastCharAccepted = ' '; }
      break;
    case 'P':
      if(c == 'D') { lastCharAccepted = c; }
      else { pursuitString = 0; lastCharAccepted = ' '; }
      break;
    case 'D':
      if(c == ',') ret = true; // this is the only way it can be true
      // unconditionally clear the state machine after ','
      pursuitString = 0; lastCharAccepted = ' ';
      break;
    default: 
      pursuitString = 0; lastCharAccepted = ' ';
      break;
    }
    break;
  case 2: // CLOSED,
    switch(lastCharAccepted){
    case 'C':
      if(c == 'L') { lastCharAccepted = c; }
      else { pursuitString = 0; lastCharAccepted = ' '; }
      break;
    case 'L':
      if(c == 'O') { lastCharAccepted = c; }
      else { pursuitString = 0; lastCharAccepted = ' '; }
      break;
    case 'O':
      if(c == 'S') { lastCharAccepted = c; }
      else { pursuitString = 0; lastCharAccepted = ' '; }
      break;
    case 'S':
      if(c == 'E') { lastCharAccepted = c; }
      else { pursuitString = 0; lastCharAccepted = ' '; }
      break;      
    case 'E':
      if(c == 'D'){
        // handle seeing CLOSED
        socket_connected = false;
        // Serial.println("socket closed because saw CLOSED");
      }
      // unconditionally clear the state machine after 'D'
      pursuitString = 0; lastCharAccepted = ' ';
      break;
    default: 
      pursuitString = 0; lastCharAccepted = ' ';
      break;
    }
    break;    
  case 3: // UNLINK,
    switch(lastCharAccepted){
    case 'U':
      if(c == 'N') { lastCharAccepted = c; pursuitDepth++; }
      else { pursuitString = 0; lastCharAccepted = ' '; }
      break;
    case 'N':
      // this is the only slightly wierd case because UNLINK has two N's
      // and it's therefore also the only reason I keep the pursuitDepth variable
      // maintaining it also doesn't matter once it increments past 2
      if(pursuitDepth == 2){ // after the first N in UNLINK
        if(c == 'L') { lastCharAccepted = c; pursuitDepth++; }
        else { pursuitString = 0; lastCharAccepted = ' '; }
      }
      else if(pursuitDepth == 5){ // after the second N in UNLINK
        if(c == 'K'){
          // handle seeing UNLINK
          socket_connected = false;
          // Serial.println("socket closed because saw UNLINK");
        }
        // unconditionally clear the state machine after 'K'
        pursuitString = 0; lastCharAccepted = ' ';
      }
      else { pursuitString = 0; lastCharAccepted = ' '; }
      break;
    case 'L':
      if(c == 'I') { lastCharAccepted = c; pursuitDepth++; }
      else { pursuitString = 0; lastCharAccepted = ' '; }
      break;
    case 'I':
      if(c == 'N') { lastCharAccepted = c; pursuitDepth++; }
      else { pursuitString = 0; lastCharAccepted = ' '; }
      break;      
    default: 
      pursuitString = 0; lastCharAccepted = ' ';
      break;
    }
    break;
  case 4:
    switch(lastCharAccepted){
    case 'W':
      if(c == 'I') { lastCharAccepted = c; pursuitDepth++; }
      else { pursuitString = 0; lastCharAccepted = ' '; }
      break;      
    case 'I':
                             //                         v v  v
      if(pursuitDepth == 2){ // after the first I in 'WIFI DISCONNECT'
                             //                       0123456789abcde
        if(c == 'F') { lastCharAccepted = c; pursuitDepth++; }
        else { pursuitString = 0; lastCharAccepted = ' '; }
      }
      else if(pursuitDepth == 4){ // after the second I in 'WIFI DISCONNECT'
        if(c == ' ') { lastCharAccepted = c; pursuitDepth++; }
        else { pursuitString = 0; lastCharAccepted = ' '; }
      }
      else if(pursuitDepth == 7){ // after the third I in 'WIFI DISCONNECT'
        if(c == 'S') { lastCharAccepted = c; pursuitDepth++; }
        else { pursuitString = 0; lastCharAccepted = ' '; }        
      }
      else { pursuitString = 0; lastCharAccepted = ' '; }
      break;
    case 'F':
      if(c == 'I') { lastCharAccepted = c; pursuitDepth++; }
      else { pursuitString = 0; lastCharAccepted = ' '; }
      break;         
    case ' ':
      if(c == 'D') { lastCharAccepted = c; pursuitDepth++; }
      else { pursuitString = 0; lastCharAccepted = ' '; }
      break;    
    case 'D':
      if(c == 'I') { lastCharAccepted = c; pursuitDepth++; }
      else { pursuitString = 0; lastCharAccepted = ' '; }
      break;      
    case 'S':
      if(c == 'C') { lastCharAccepted = c; pursuitDepth++; }
      else { pursuitString = 0; lastCharAccepted = ' '; }
      break;
    case 'C':                  //                                v    v
      if(pursuitDepth == 0x9){ // after the first C in 'WIFI DISCONNECT'
                               //                       0123456789abcde
        if(c == 'O') { lastCharAccepted = c; pursuitDepth++; }
        else { pursuitString = 0; lastCharAccepted = ' '; }
      }
      else if(pursuitDepth == 0xe){ // after the second C in 'WIFI DISCONNECT'
        if(c == 'T') { 
          // this is a goal state
          wifi_is_connected = false;
        }
        // unconditionally clear the state machine after 'T'
        pursuitString = 0; lastCharAccepted = ' ';
      }
      else { pursuitString = 0; lastCharAccepted = ' '; }
      break;                                
    case 'O':
      if(c == 'N') { lastCharAccepted = c; pursuitDepth++; }
      else { pursuitString = 0; lastCharAccepted = ' '; }
      break;
    case 'N':                  //                                  vv
      if(pursuitDepth == 0xb){ // after the first N in 'WIFI DISCONNECT'
                               //                       0123456789abcde
        if(c == 'N') { lastCharAccepted = c; pursuitDepth++; }
        else { pursuitString = 0; lastCharAccepted = ' '; }
      }
      else if(pursuitDepth == 0xc){ // after the second N in 'WIFI DISCONNECT'
        if(c == 'E') { lastCharAccepted = c; pursuitDepth++; }
        else { pursuitString = 0; lastCharAccepted = ' '; }
      }
      else { pursuitString = 0; lastCharAccepted = ' '; }
      break;          
    case 'E':
      if(c == 'C') { lastCharAccepted = c; pursuitDepth++; }
      else { pursuitString = 0; lastCharAccepted = ' '; }
      break;       
    default: 
      pursuitString = 0; lastCharAccepted = ' ';
      break;
    }
    break;
  default:
    pursuitString = 0; lastCharAccepted = ' ';
    break;
  }
  
  return ret;
}
