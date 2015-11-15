#include <ESP8266_AT_Client.h>
#include <string.h>

//#define ESP8266_AT_CLIENT_ENABLE_DEBUG
//#define ESP8266_AT_CLIENT_DEBUG_ECHO_EVERYTHING

Stream * ESP8266_AT_Client::debugStream = NULL;

ESP8266_AT_Client::ESP8266_AT_Client(uint8_t enable_pin){
  this->stream = &Serial; // default assumption
  this->socket_connected = false;
  this->input_buffer = NULL;
  this->input_buffer_length = 0;  
  this->input_buffer_read_ptr = NULL;
  this->input_buffer_write_ptr = NULL;
  this->input_buffer_tail_ptr = NULL;  
  this->enable_pin = enable_pin;
  this->num_consumed_bytes_in_input_buffer = 0;
  this->num_free_bytes_in_input_buffer = 0;
}

ESP8266_AT_Client::ESP8266_AT_Client(uint8_t enable_pin, Stream * stream){
  this->stream = stream;
  this->socket_connected = false;
  this->input_buffer = NULL;
  this->input_buffer_length = 0;    
  this->input_buffer_read_ptr = NULL;
  this->input_buffer_write_ptr = NULL;
  this->input_buffer_tail_ptr = NULL;    
  this->enable_pin = enable_pin;
  this->num_consumed_bytes_in_input_buffer = 0;
  this->num_free_bytes_in_input_buffer = 0; 
}

ESP8266_AT_Client::ESP8266_AT_Client(uint8_t enable_pin, Stream * stream, uint8_t * buf, uint16_t buf_length){
  this->stream = stream; 
  this->socket_connected = false;
  this->input_buffer = buf;
  this->input_buffer_length = buf_length; 
  this->input_buffer_read_ptr = buf;
  this->input_buffer_write_ptr = buf;
  this->input_buffer_tail_ptr = &(buf[buf_length-1]);
  this->enable_pin = enable_pin;  
  this->num_consumed_bytes_in_input_buffer = 0;  
  this->num_free_bytes_in_input_buffer = 0;  
}

boolean ESP8266_AT_Client::reset(void){  
   // reset the buffer state pointers
   input_buffer_read_ptr = input_buffer;
   input_buffer_write_ptr = input_buffer;
   input_buffer_tail_ptr = &(input_buffer[input_buffer_length-1]);
   num_consumed_bytes_in_input_buffer = 0;
   num_free_bytes_in_input_buffer = input_buffer_length;
   
   pinMode(enable_pin, OUTPUT);  
   digitalWrite(enable_pin, LOW);    
   delay(50);
   digitalWrite(enable_pin, HIGH);    
  
   ESP8266_AT_Client::DEBUG("ESP8266 Hello World.");

   if(readStreamUntil("ready", 1000)){
     ESP8266_AT_Client::DEBUG("Received 'ready'"); 
   } 
   else{
      ESP8266_AT_Client::DEBUG("Panic"); 
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
int ESP8266_AT_Client::connect(IPAddress ip, uint16_t port){
  char host[16] = {0}; // worst case 111.111.111.111 + the null terminator = 16 characters
  snprintf(host, 15, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
  return connect(host, port);
}

/** Connect to server by hostname
	@param host			the host name to resolve to an IP address and connect to
	@param port			the port to connect to
	@return returns 0 if last command is still executing, 1 success, 2 if there are no resources
 */
int ESP8266_AT_Client::connect(const char *host, uint16_t port){
  // only implementing a blocking API - never return 0
  // set up an AT command and send it
  // then return whether or not it succeeded
  int ret = 2; // initialize to error
  socket_connected = false;  
  num_characters_remaining_to_receive = 0;
  
  ESP8266_AT_Client::DEBUG("Connecting to ", (char *) host);
  
  flushInput();
  stream->print("AT+CIPSTART=\"TCP\",\"");
  stream->print(host);
  stream->print("\",");
  stream->print(port);
  stream->print(",");
  stream->print(120); // keep alive interval in units of "0.5 second intervals" (i.e. 1 minute)
  stream->print("\r\n");  
  
  // ESP8266 responds with either "OK", "ERROR", or "ALREADY CONNECT" 
  clearTargetMatchArray();
  addStringToTargetMatchList("OK");
  addStringToTargetMatchList("ERROR");
  addStringToTargetMatchList("ALREADY CONNECT");
  uint8_t match_index = 0xFF;  
  if(readStreamUntil(&match_index)){
     if(match_index == 0){ // got "OK"     
       ESP8266_AT_Client::DEBUG("TCPConnect Connected");     
       ret = 1; // success
     }  
     else if(match_index == 2){
       ESP8266_AT_Client::DEBUG("TCPConnect Already Connected");      
       ret = 1; // success     
     }
     else{     
       ESP8266_AT_Client::DEBUG("TCPConnect Failed");
       ret = 2; // error
     }     
  }
  
  if(ret == 1){
    socket_connected = true;
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
  
  stream->print("AT+CIPSEND=");
  stream->print(sz);
  stream->print("\r\n");

  if(readStreamUntil(">", 1000)){  
    ret = stream->write(buf, sz); // pass it along    
    
    if(readStreamUntil("SEND OK\r\n\r\n", 1000)){
       ESP8266_AT_Client::DEBUG("Send succeeded");     
    }
  }
  
  return ret;
}

/** Check if there is data pending receipt
	@return 1 if exists, 0 if not exists
 */
int ESP8266_AT_Client::available(){
  // an intent to read() is always preceded by a call to available(),
  // if the caller knows what is good for them,
  // so this is where we need to perform the receive 
  // in a synchronous manner (with timeout)
  
  // some state management needs to be encapsulated here as we'll only read one
  // packet in at a time and we'll keep a 1500 byte buffer around to store
  // the buffered data, hey maybe we can even implement peek correctly
  int ret = num_consumed_bytes_in_input_buffer;  
      
  if(ret == 0){  
    // if there's nothing available and we're connnected
    // lets go ahead and try and get some data buffered for processing
    if(socket_connected){
      receive(); // doesn't really matter what the response is
      ret = num_consumed_bytes_in_input_buffer;
//      switch(receive()){
//        case 1: // input buffer ready        
//        case 3: // timeout        
//          break;
//        case 2: // connection closed
//          // socket_connected = false;
//          break;
//        case 4: // sync error
//          // stop();
//          break;
//      }    

    }
  }

  return ret;
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
  if(socket_connected){

    // set up an AT command and send it
    // then return whether or not it succeeded
    
    flushInput();
    stream->print("AT+CIPCLOSE");
    stream->print("\r\n");   
    
    // ESP8266 responds with either "OK", "ERROR"
    clearTargetMatchArray();
    addStringToTargetMatchList("CLOSED");
    addStringToTargetMatchList("ERROR");

    uint8_t match_index = 0xFF;        
    if(readStreamUntil(&match_index, 5000)){
       if(match_index == 0){ // got "CLOSED"
         ESP8266_AT_Client::DEBUG("TCPClose Succeeded");
       }  
       else{       
         ESP8266_AT_Client::DEBUG("TCPClose Failed");
       }     
    }
    
  }
    
  socket_connected = false;  
}

/** Check if connected to server
	@return 1 if connected
 */

uint8_t ESP8266_AT_Client::connected(){
  return connected(true);
} 
 
uint8_t ESP8266_AT_Client::connected(boolean actively_check){
  uint8_t ret = 1; // assume we are connected
  if(!socket_connected){
    return 0;
  }
  
  if(socket_connected && actively_check){
    // set up an AT command and send it
    // then return whether or not it succeeded

    clearTargetMatchArray();
    addStringToTargetMatchList("STATUS:2"); // got ip    
    addStringToTargetMatchList("STATUS:3"); // connected
    addStringToTargetMatchList("STATUS:4"); // disconnected
    addStringToTargetMatchList("OK\r\n");  
    addStringToTargetMatchList("+IPD,");    
        
    // you have to get "STATUS:" and a numeric code
    // then you have *may* get "+CIPSTATUS:"
    // then you get "OK" 
    if(num_characters_remaining_to_receive > 0){
      receive(true); // receive is already in progress    
    }
    else{    
      stream->print("AT+CIPSTATUS");
      stream->print("\r\n");  
        
      uint8_t match_index = 0xFF;      
      if(readStreamUntil(&match_index, 100)){      
        switch(match_index){
        case 2: //disconnected
          socket_connected = false;
          ret = 0;   
          break;
        case 4: // +IPD, gotta deal with it
          receive(true);
          break;
        case 0: 
        case 1:
        case 3:
        default:
           // nothing to do
          break;
        }                             
      }          
      
      if(match_index != 3){ 
        if(readStreamUntil(&match_index, 100)){
          if(match_index != 3){ //  != "OK"
             ESP8266_AT_Client::DEBUG("CIPSTATUS not OK"); 
             // doesn't necessarily mean we're disconnected
          }
        }
        else{
          DEBUG("AT+CIPSTATUS expected OK, but didn't receive it");
        }          
      }  
    }  

  }  
  
  return ret;
}

uint8_t ESP8266_AT_Client::connectedToNetwork(void){
  uint8_t ret = 0; 

  clearTargetMatchArray();
  addStringToTargetMatchList("+CWJAP:"); // connected
  addStringToTargetMatchList("No AP"); // disconnected
  addStringToTargetMatchList("OK\r\n");    
  
  stream->print("AT+CWJAP_CUR?");
  stream->print("\r\n");  
  
  uint8_t match_index = 0xFF;  
  if(readStreamUntil(&match_index, 100)){   
    clearTargetMatchArray();
  }
  
  return ret;  
}

boolean ESP8266_AT_Client::getIPAddress(char * ip_str){
  boolean ret = false;
  
  clearTargetMatchArray();
  addStringToTargetMatchList("+CIFSR:STAIP,\""); // connected
  addStringToTargetMatchList("OK\r\n");    
  addStringToTargetMatchList("ERROR\r\n");    
  
  flushInput();
  stream->print("AT+CIFSR");
  stream->print("\r\n");    
  
  uint8_t match_index = 0xFF;  
  if(readStreamUntil(&match_index, 100)){   
    if(match_index == 0){
      char tmp[32] = {0};
      if(readStreamUntil("\"", &(tmp[0]), 32, 10)){
        strncpy(ip_str, tmp, 16); // an ip address is at most 15 characters long
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

boolean ESP8266_AT_Client::getIPAddress(uint32_t * ip){
  boolean ret = false;
  char tmp[16] = {0};
  if(getIPAddress((char *) tmp)){
    char * token = strtok(tmp, ".");
    uint8_t num_tokens = 0;
    uint32_t ip_address = 0;
    
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
  }

  
  return ret;
}

boolean ESP8266_AT_Client::getMacAddress(char * mac_str){
  boolean ret = false;
  
  clearTargetMatchArray();
  addStringToTargetMatchList("+CIFSR:STAMAC,\""); // connected
  addStringToTargetMatchList("OK\r\n");    
  addStringToTargetMatchList("ERROR\r\n");    
  
  flushInput();
  stream->print("AT+CIFSR");
  stream->print("\r\n"); 
  
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
  char local_mac[6] = {0};
  if(getMacAddress((char *) tmp)){
    char * token = strtok(tmp, ":");
    uint8_t num_tokens = 0;
    uint32_t ip_address = 0;
    
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
  }

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
boolean ESP8266_AT_Client::readStreamUntil(uint8_t * match_idx, char * target_buffer, uint16_t target_buffer_length, int32_t timeout_ms){
  boolean match_found = false;
  const uint8_t local_target_buffer_length = 32;
  static boolean initial_call = true;
  static char local_target_buffer[local_target_buffer_length] = {0};
  static uint16_t local_target_buffer_index = 0;
  static uint8_t match_char_idx[ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES] = {0};  
  unsigned long previousMillis = millis();      
  
#ifdef ESP8266_AT_CLIENT_ENABLE_DEBUG
  ESP8266_AT_Client::DEBUG("+++");
  for(uint8_t ii = 0; ii < ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES; ii++){
    uint16_t target_match_length = target_match_lengths[ii];
    if(target_match_length == 0){ 
      break;
    }
    ESP8266_AT_Client::DEBUG("  Waiting for ", target_match_array[ii]);  
  }
  ESP8266_AT_Client::DEBUG("===");    
#endif  
  
  if(initial_call){
    initial_call = false;
    //ESP8266_AT_Client::debugStream->println("\nbegin>");
  }
  
  while(!match_found){ // until a match is found
    unsigned long currentMillis = millis();
    if((timeout_ms > 0) && (currentMillis - previousMillis >= timeout_ms)){
       break;
    }

    if(stream->available()){
      previousMillis = millis(); // reset the timeout
      char chr = stream->read(); // read a character

#ifdef ESP8266_AT_CLIENT_DEBUG_ECHO_EVERYTHING     
      ESP8266_AT_Client::debugStream->print(chr); // echo the received characters to the Serial Monitor
#endif
      
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
              ESP8266_AT_Client::DEBUG("Warn: caller's buffer is smaller than needed to contain", local_target_buffer);
            }
            strncpy(target_buffer, local_target_buffer, target_buffer_length);
          }
          
          // reset the stateful variables          
          memset(match_char_idx, 0, ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES);
          memset(local_target_buffer, 0, 32);
          local_target_buffer_index = 0;   
          initial_call = true;                     
          //ESP8266_AT_Client::debugStream->println("<end");
          break;
        }        
      }
      
      if(!match_found && target_buffer != NULL){
        if(local_target_buffer_index < local_target_buffer_length - 1){
          local_target_buffer[local_target_buffer_index++] = chr;
        }
        else{
          ESP8266_AT_Client::DEBUG("Local target buffer would overflow");
          break; // target buffer overflow
        }
      }      
    }
  }
  
  ESP8266_AT_Client::DEBUG("*** ", (uint8_t) match_found);
   
  return match_found;   
}

// pass a single string to match against
// the string must not be longer than 31 characters
boolean ESP8266_AT_Client::readStreamUntil(char * target_match, char * target_buffer, uint16_t target_buffer_length, int32_t timeout_ms){  
  uint8_t dummy_return;
  
  if(strlen(target_match) > 31){
    return false; 
  }
  else{
    clearTargetMatchArray();
    addStringToTargetMatchList(target_match);
    return readStreamUntil(&dummy_return, target_buffer, target_buffer_length, timeout_ms);
  }
}

boolean ESP8266_AT_Client::readStreamUntil(char * target_match, int32_t timeout_ms){
  return readStreamUntil(target_match, NULL, 0, timeout_ms);
}

boolean ESP8266_AT_Client::readStreamUntil(char * target_match){
  return readStreamUntil(target_match, -1);
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
    
    //ESP8266_AT_Client::debugStream->write(c);    
       
    return true;
  }
  else{
    ESP8266_AT_Client::DEBUG("Input Buffer Underflow!");
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

    //ESP8266_AT_Client::debugStream->write(ret);
    
    return ret;
  }
  
  return -1;
}

void ESP8266_AT_Client::flushInput(){
  while(num_consumed_bytes_in_input_buffer > 0){ 
    char chr = stream->read();
    
#ifdef ESP8266_AT_CLIENT_DEBUG_ECHO_EVERYTHING    
    ESP8266_AT_Client::debugStream->println((uint8_t) chr, HEX); // echo the received characters to the Serial Monitor   
#endif    

  }  
}

// 1 is Station Mode
// 2 is SoftAP Mode
// 3 is SoftAP + Station Mode
boolean ESP8266_AT_Client::setNetworkMode(uint8_t mode){
  boolean ret = false;
  
  flushInput();  
  stream->print("AT+CWMODE_CUR=");
  stream->print(mode);
  stream->print("\r\n");    

  // ESP8266 responds with either "OK", "ERROR"
  clearTargetMatchArray();
  addStringToTargetMatchList("OK");
  addStringToTargetMatchList("ERROR");
  
  uint8_t match_index = 0xFF;
  if(readStreamUntil(&match_index)){
     if(match_index == 0){
       ESP8266_AT_Client::DEBUG("Debug: NetworkMode Succeeded");
       ret = true;
     }  
     else{
       ESP8266_AT_Client::DEBUG("Debug: NetworkMode Failed");
       ret = false;
     }     
  }
  
  flushInput();
  return ret;    
}

boolean ESP8266_AT_Client::connectToNetwork(char * ssid, char * pwd, int32_t timeout_ms, void (*onConnect)(void)){
  flushInput();
  stream->print("AT+CWJAP_CUR=\"");
  stream->print(ssid);
  stream->print("\",\"");
  stream->print(pwd);
  stream->print("\"");
  stream->print("\r\n");
  
  // wait for connected status
  if(readStreamUntil("WIFI CONNECTED", timeout_ms)){

    ESP8266_AT_Client::DEBUG("Connected to Network"); 
    if(onConnect != NULL){
      onConnect();
    }
    // wait for got IP status
    if(readStreamUntil("WIFI GOT IP", 60000)){
       ESP8266_AT_Client::DEBUG("Got IP"); 
 
       if(readStreamUntil("OK")){
         return true;
       }      
    }
    else{
       ESP8266_AT_Client::DEBUG("Failed to get IP address"); 
       return false;
    }      
  }
  else{  
     ESP8266_AT_Client::DEBUG("Failed to connect to Network");    
     return false;    
  }
  
  return false;
}

//TODO: Implement
boolean ESP8266_AT_Client::disconnectFromNetwork(){
  boolean ret = false;
  
  flushInput();
  stream->print("AT+CWQAP");
  stream->print("\r\n");  
  
  if(readStreamUntil("WIFI DISCONNECT", 1000)){
     ESP8266_AT_Client::DEBUG("Disconnected from Network");    
     ret = true;
  }
  else{
     ESP8266_AT_Client::DEBUG("Failed to disconnect from Network");
  }

  return ret;
}

void ESP8266_AT_Client::receive(boolean delegate_received_IPD){    
  static enum {WAITING_FOR_IPD, WAITING_FOR_IPD_OR_CLOSED, WAITING_FOR_COLON, PROCESSING_IPD} receive_state = WAITING_FOR_IPD;
  uint8_t match_index = 0xFF;
    
  if(num_free_bytes_in_input_buffer == 0){
    // trying to receive more is unproductive until we get rid of some that we alrady have
    return;
  }
  
  if(stream->available() == 0){
    // there's nothing in the hardware buffer to digest
    return;
  }
      
  if(delegate_received_IPD){
    if(receive_state == WAITING_FOR_IPD || receive_state == WAITING_FOR_IPD_OR_CLOSED){
      receive_state = WAITING_FOR_COLON;
      ESP8266_AT_Client::DEBUG("Rx State = WAITING_FOR_COLON (1)");
    }
    else{
      ESP8266_AT_Client::DEBUG("Unexpected delegate call to receive");
    }    
  }  
    
  if(receive_state == WAITING_FOR_IPD){
    clearTargetMatchArray();
    addStringToTargetMatchList("+IPD,");
    addStringToTargetMatchList("OK");
    if(readStreamUntil(&match_index, 10) && (match_index == 0)){
      // we got +IPD,
      receive_state = WAITING_FOR_COLON;
      ESP8266_AT_Client::DEBUG("Rx State = WAITING_FOR_COLON (2)");
    }
  }
  else if(receive_state == WAITING_FOR_IPD_OR_CLOSED){
    clearTargetMatchArray();
    addStringToTargetMatchList("+IPD,");    
    addStringToTargetMatchList("CLOSED");   
    addStringToTargetMatchList("OK");
    if(readStreamUntil(&match_index, 10)){
      if(match_index == 0){      
        // we got +IPD,
        receive_state = WAITING_FOR_COLON;
        ESP8266_AT_Client::DEBUG("Rx State = WAITING_FOR_COLON (3)");
      }
      else if(match_index == 1){
        // we got CLOSED
        socket_connected = false;        
        receive_state = WAITING_FOR_IPD;
        ESP8266_AT_Client::DEBUG("Rx State = WAITING_FOR_IPD");
        return; // we're done here
      }
    }      
  }  
 
  if(receive_state == WAITING_FOR_COLON){
    clearTargetMatchArray();
    addStringToTargetMatchList(":");    
    char tmp[32] = {0};
    if(readStreamUntil(&match_index, &(tmp[0]), 32, 10)){
      if(match_index == 0){
        char * temp = NULL;
        uint32_t num_characters_expected = strtoul((char *) tmp, &temp, 10);
        if (*temp != '\0'){
          DEBUG("Debug: Receive Error, length parse error on ", temp);  
          //TODO: this is a disaster... caller should be able to know about it if it happens
          receive_state = WAITING_FOR_IPD_OR_CLOSED;
          ESP8266_AT_Client::DEBUG("Rx State = WAITING_FOR_IPD_OR_CLOSED (1)");
        }
        else{
          num_characters_remaining_to_receive = num_characters_expected;
          receive_state = PROCESSING_IPD;
          ESP8266_AT_Client::DEBUG("Rx State = PROCESSING_IPD");
        }
      }
    }
  }  
    
  if(receive_state == PROCESSING_IPD){
    ESP8266_AT_Client::DEBUG("Remaining: ", num_characters_remaining_to_receive);
    uint32_t bytes_read_this_cycle = 0;
    while((num_characters_remaining_to_receive > 0) && (num_free_bytes_in_input_buffer > 0) && stream->available()){
      uint8_t ch = stream->read();
      
      bytes_read_this_cycle++;
#ifdef ESP8266_AT_CLIENT_DEBUG_ECHO_EVERYTHING
      ESP8266_AT_Client::debugStream->write(ch);
#endif
      
      if(writeToInputBuffer(ch)){            
        num_characters_remaining_to_receive--;
      }      
      else{
        ESP8266_AT_Client::DEBUG("writeToInputBuffer failed");
        break;
      }
    }
    
    if(num_characters_remaining_to_receive == 0){
      receive_state = WAITING_FOR_IPD_OR_CLOSED;
      ESP8266_AT_Client::DEBUG("Rx State = WAITING_FOR_IPD_OR_CLOSED (2)");
    }    
  }
}

void ESP8266_AT_Client::DEBUG(char * msg){
#ifdef ESP8266_AT_CLIENT_ENABLE_DEBUG       
  ESP8266_AT_Client::debugStream->print("Debug: ");     
  ESP8266_AT_Client::debugStream->println(msg);
#endif
}

void ESP8266_AT_Client::DEBUG(char * msg, uint16_t value){
#ifdef ESP8266_AT_CLIENT_ENABLE_DEBUG  
  ESP8266_AT_Client::debugStream->print("Debug: ");     
  ESP8266_AT_Client::debugStream->print(msg);
  ESP8266_AT_Client::debugStream->println(value);
#endif
}

void ESP8266_AT_Client::DEBUG(char * msg, char * value){
#ifdef ESP8266_AT_CLIENT_ENABLE_DEBUG  
  ESP8266_AT_Client::debugStream->print("Debug: ");     
  ESP8266_AT_Client::debugStream->print(msg);
  ESP8266_AT_Client::debugStream->print("\"");
  ESP8266_AT_Client::debugStream->print(value);
  ESP8266_AT_Client::debugStream->println("\"");
#endif
}

void ESP8266_AT_Client::clearTargetMatchArray(void){
  for(uint8_t ii = 0; ii < ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES; ii++){
    target_match_array[ii][0] = NULL;
    target_match_lengths[ii] = 0;
  }
}
