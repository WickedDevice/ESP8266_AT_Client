#include <ESP8266_AT_Client.h>
#include <string.h>

//#define ESP8266_AT_CLIENT_ENABLE_DEBUG
//#define ESP8266_AT_CLIENT_DEBUG_ECHO_EVERYTHING

Stream * ESP8266_AT_Client::debugStream = NULL;
boolean ESP8266_AT_Client::debug_echo_everything_enable = false;

ESP8266_AT_Client::ESP8266_AT_Client(uint8_t enable_pin){
  this->stream = NULL;
  this->socket_connected = false;
  this->input_buffer = NULL;
  this->input_buffer_length = 0;  
  this->input_buffer_read_ptr = NULL;
  this->input_buffer_write_ptr = NULL;
  this->input_buffer_tail_ptr = NULL;  
  this->enable_pin = enable_pin;
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
}

ESP8266_AT_Client::ESP8266_AT_Client(uint8_t enable_pin, Stream * stream, uint8_t * buf, uint32_t buf_length){
  this->stream = stream; 
  this->socket_connected = false;
  this->input_buffer = buf;
  this->input_buffer_length = buf_length; 
  this->input_buffer_read_ptr = buf;
  this->input_buffer_write_ptr = buf;
  this->input_buffer_tail_ptr = &(buf[buf_length-1]);
  this->enable_pin = enable_pin;    
}

boolean ESP8266_AT_Client::reset(void){
  
   // reset the buffer state pointers
   this->input_buffer_read_ptr = this->input_buffer;
   this->input_buffer_write_ptr = this->input_buffer;
   this->input_buffer_tail_ptr = &(this->input_buffer[this->input_buffer_length-1]);

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

void ESP8266_AT_Client::setInputBuffer(uint8_t * buf, uint32_t buf_length){
  this->input_buffer = buf;
  this->input_buffer_length = buf_length;
  this->input_buffer_read_ptr = buf;
  this->input_buffer_write_ptr = buf;
  this->input_buffer_tail_ptr = &(buf[buf_length-1]);     
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
  this->socket_connected = false;  
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
  static char target_match_array[4][ESP8266_AT_CLIENT_MAX_STRING_LENGTH + 1] = {0};  
  uint8_t match_index = 0xFF;
  ESP8266_AT_Client::addStringToList(target_match_array, "OK", 4);
  ESP8266_AT_Client::addStringToList(target_match_array, "ERROR", 4);
  ESP8266_AT_Client::addStringToList(target_match_array, "ALREADY CONNECT", 4);
  if(readStreamUntil(target_match_array, &match_index)){
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
    this->socket_connected = true;
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
  
  flushInput();
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
  // if the call knows what is good for them,
  // so this is where we need to perform the receive 
  // in a synchronous manner (with timeout)
  
  // some state management needs to be encapsulated here as we'll only read one
  // packet in at a time and we'll keep a 1500 byte buffer around to store
  // the buffered data, hey maybe we can even implement peek correctly
  int ret = 0;  
  
  if(input_buffer_write_ptr >= input_buffer_read_ptr){
    // write pointer to read pointer (exclusive)
    ret = input_buffer_write_ptr - input_buffer_read_ptr;
  }
  else{
    // tail to the read pointer (inclusive) + write pointer to head (exclusive)
    ret = (input_buffer_tail_ptr - input_buffer_read_ptr + 1) + (input_buffer_write_ptr - input_buffer);
  }
  
  if(ret == 0){  
    // if there's nothing available and we're connnected
    // lets go ahead and try and get some data buffered for processing
    if(socket_connected){
      receive(10); // doesn't really matter what the response is
      
//      switch(receive(10)){
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
  int num_bytes_read = 0;
  while((num_bytes_read < size) && (available() > 0)){
    buf[num_bytes_read++] = readFromInputBuffer();
  }
  
  return num_bytes_read;
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
    char target_match_array[3][ESP8266_AT_CLIENT_MAX_STRING_LENGTH + 1] = {0};  
    uint8_t match_index = 0xFF;
    ESP8266_AT_Client::addStringToList(target_match_array, "CLOSED", 3);
    ESP8266_AT_Client::addStringToList(target_match_array, "ERROR", 3);
    if(readStreamUntil(target_match_array, &match_index, 5000)){
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
  uint8_t ret = 0;
  
  if(socket_connected){
    // set up an AT command and send it
    // then return whether or not it succeeded

    char target_match_array[3][ESP8266_AT_CLIENT_MAX_STRING_LENGTH + 1] = {0};  
    uint8_t match_index = 0xFF;
    ESP8266_AT_Client::addStringToList(target_match_array, "+CIPSTATUS:", 3);
    ESP8266_AT_Client::addStringToList(target_match_array, "OK\r\n", 3);  
        
    flushInput();
    stream->print("AT+CIPSTATUS");
    stream->print("\r\n");
  
    // then you have to get "+CIPSTATUS:"
    // then you get "OK" 
    // if you're connected
  
    if(readStreamUntil(target_match_array, &match_index, 100)){    
      if(match_index == 0){ // got "+CIPSTATUS:"
        if(readStreamUntil(target_match_array, &match_index, 100)){
          if(match_index == 1){ // got "OK"
            ret = 1;  
          }
        }        
      }
      else{ 
        socket_connected = false;
        ret = 0;
      }
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
boolean ESP8266_AT_Client::addStringToList(char list[][ESP8266_AT_CLIENT_MAX_STRING_LENGTH+1], char * str, uint8_t max_num_entries){
  uint16_t free_index = 0xFFFF;
  
  if(strlen(str) <= ESP8266_AT_CLIENT_MAX_STRING_LENGTH){
    
    // search the list for an empty space
    // the last empty space must remain empty
    // so don't include it in the search
    for(uint16_t ii = 0; ii < (max_num_entries - 1); ii++){      
      uint16_t len = strlen(list[ii]);  
      if(0 == len){
        free_index = ii;
        break;
      }
    }
        
    // free index is the firs empty space in the list
    // or 0xFFFF if no free entries were found
    
    // if free index points to a viable position
    // then copy the candidate string into that position
    // and limit the number of characters copied
    if(free_index < (max_num_entries - 1)){         
      char * tgt_addr = list[free_index];
      memcpy(tgt_addr, 0, ESP8266_AT_CLIENT_MAX_STRING_LENGTH+1); // fill with NULLs
      strncpy(tgt_addr, str, ESP8266_AT_CLIENT_MAX_STRING_LENGTH);  // copy the string in      
      return true;
    }
    
  }
  
  return false;
}

// pass in an array of strings to match against
// the list is presumed to be terminated by a NULL string
// this function can only handle up to ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES matching targets
boolean ESP8266_AT_Client::readStreamUntil(char target_match[][ESP8266_AT_CLIENT_MAX_STRING_LENGTH+1], uint8_t * match_idx, char * target_buffer, uint16_t target_buffer_length, int32_t timeout_ms){
  boolean match_found = false;
  uint32_t target_buffer_index = 0;
  static uint8_t match_char_idx[ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES] = {0};
  unsigned long previousMillis = millis();
  
  memset(match_char_idx, 0, ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES);
  
  ESP8266_AT_Client::DEBUG("+++");
  for(uint8_t ii = 0; ii < ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES; ii++){
    uint16_t target_match_length = strlen(target_match[ii]);
    if(target_match_length == 0){ 
      break;
    }
    ESP8266_AT_Client::DEBUG("Waiting for ", target_match[ii]);
  }
  ESP8266_AT_Client::DEBUG("===");
  
  while(!match_found){ // until a match is found
    unsigned long currentMillis = millis();
    if((timeout_ms > 0) && (currentMillis - previousMillis >= timeout_ms)){
//       ESP8266_AT_Client::DEBUG("Timeout1");
       break;
    }
  
    if(stream->available()){
      previousMillis = millis(); // reset the timeout
      char chr = stream->read(); // read a character

#ifdef ESP8266_AT_CLIENT_DEBUG_ECHO_EVERYTHING
      if(debugStream != NULL && ESP8266_AT_Client::debug_echo_everything_enable){
         ESP8266_AT_Client::debugStream->print(chr); // echo the received characters to the Serial Monitor
      }
#endif
      
      // for each target match
      for(uint8_t ii = 0; ii < ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES; ii++){
        uint16_t target_match_length = strlen(target_match[ii]);
        // an empty string in the list signals the end of the list
        if(target_match_length == 0){ 
          break;
        } 
        
        // if the current character is a match for this string
        // advance it's match index, 
        // otherwise reset its match index
        if(chr == target_match[ii][match_char_idx[ii]]){
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
          break;
        }        
      }
      
      if(!match_found && target_buffer != NULL){
        if(target_buffer_index < target_buffer_length){
          target_buffer[target_buffer_index++] = chr;
        }
        else{
          ESP8266_AT_Client::DEBUG("Target buffer overflow");
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
  char target_match_array[2][ESP8266_AT_CLIENT_MAX_STRING_LENGTH + 1] = {0};
  uint8_t dummy_return;
  
  if(strlen(target_match) > 31){
    return false; 
  }
  else{
    ESP8266_AT_Client::addStringToList(target_match_array, target_match, 2);
    return readStreamUntil(target_match_array, &dummy_return, target_buffer, target_buffer_length, timeout_ms);
  }
}

boolean ESP8266_AT_Client::readStreamUntil(char * target_match, int32_t timeout_ms){
  return readStreamUntil(target_match, NULL, 0, timeout_ms);
}

boolean ESP8266_AT_Client::readStreamUntil(char * target_match){
  return readStreamUntil(target_match, -1);
}

boolean ESP8266_AT_Client::readStreamUntil(char target_match[][ESP8266_AT_CLIENT_MAX_STRING_LENGTH+1], uint8_t * match_idx, int32_t timeout_ms){
  return readStreamUntil(target_match, match_idx, NULL, 0, timeout_ms);
}

boolean ESP8266_AT_Client::readStreamUntil(char target_match[][ESP8266_AT_CLIENT_MAX_STRING_LENGTH+1], uint8_t * match_idx){
  return readStreamUntil(target_match, match_idx, -1);  
}

boolean ESP8266_AT_Client::readStream(uint32_t num_characters_expected, int32_t timeout_ms){
  uint32_t num_characters_read = 0;
  unsigned long previousMillis = millis();   
  
  while(num_characters_read < num_characters_expected){
    unsigned long currentMillis = millis();    
    if((timeout_ms > 0) && (currentMillis - previousMillis >= timeout_ms)){
//       ESP8266_AT_Client::DEBUG("Timeout2");
       return false; // timeout condition
    }
    if(stream->available()){
      while(stream->available()){
        previousMillis = millis(); // reset the timeout      
        char chr = stream->read();

#ifdef ESP8266_AT_CLIENT_DEBUG_ECHO_EVERYTHING   
        if(debugStream != NULL && ESP8266_AT_Client::debug_echo_everything_enable){
           ESP8266_AT_Client::debugStream->print(chr); // echo the received characters to the Serial Monitor
        }
#endif
        
        if(!writeToInputBuffer(chr)){
          return false; // buffer overflow!
        }
        num_characters_read++;
        
        if(num_characters_read == num_characters_expected){
          return true; // buffer is full 
        }
      }
    }
  }
}

// writes c to the write pointer in the input buffer
// if advancing the write pointer would collide with 
// the read pointer, the write pointer is not advanced
// otherwise the write pointer is advanced
boolean ESP8266_AT_Client::writeToInputBuffer(uint8_t c){
  uint8_t * tmp = input_buffer_write_ptr;
  
  *tmp = c; // write the value to the buffer
  
  // update the write pointer to the next location to write to within the buffer
  if(tmp == input_buffer_tail_ptr){
    input_buffer_write_ptr = input_buffer;
  }
  else{
    input_buffer_write_ptr++;
  }
  
  // if write *now* points at read, this is an overflow and we should revert it
  // because the reader of the buffer is not keepign up
  // and indicate an overflow condition 
  if(input_buffer_write_ptr == input_buffer_read_ptr){
    input_buffer_write_ptr = tmp;
    return false;
  }
  
  return true; // successful add
}

// returns the character at the current read pointer
// if the read pointer currently points to the write pointer
// the read pointer is not advanced
// otherwise the read pointer is advanced
uint8_t ESP8266_AT_Client::readFromInputBuffer(void){
  uint8_t ret = *input_buffer_read_ptr;
  
  // if the read pointer is not pointing at the write pointer
  if(input_buffer_read_ptr != input_buffer_write_ptr){
    // update the read pointer to the next location to read from within the buffer  
    if(input_buffer_read_ptr == input_buffer_tail_ptr){
      input_buffer_read_ptr = input_buffer;      
    }
    else{
      input_buffer_read_ptr++;
    }
  }
  return ret;
}

void ESP8266_AT_Client::flushInput(){
  receive(10);
  while(stream->available()){    
    char chr = stream->read();
    
#ifdef ESP8266_AT_CLIENT_DEBUG_ECHO_EVERYTHING    
    if(debugStream != NULL && ESP8266_AT_Client::debug_echo_everything_enable){
       ESP8266_AT_Client::debugStream->println((uint8_t) chr, HEX); // echo the received characters to the Serial Monitor
    }
    
#endif    

  }
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

// returns 1 if input buffer is ready, 2 if connection closed, 3 if timeout, 4 on sync error
// the first three reasons (1, 2, 3) are kind of normal exits
// reason 3 (timeout) is kind of bad, and *might* be a good reason to force close the socket and start over
// the last reason (sync error) is really bad news - you should probably force close the socket and start over
uint8_t ESP8266_AT_Client::receive(int32_t timeout){
  uint8_t ret = 0;
  static char target_match_array[4][ESP8266_AT_CLIENT_MAX_STRING_LENGTH + 1] = {0};    
  uint8_t match_index = 0xFF;
  
  ESP8266_AT_Client::addStringToList(target_match_array, "+IPD,", 4);
  ESP8266_AT_Client::addStringToList(target_match_array, "CLOSED", 4);
  ESP8266_AT_Client::addStringToList(target_match_array, "OK", 4);
  while(ret == 0){
    static char tmp[32] = {0};
      
    if(readStreamUntil(target_match_array, &match_index, timeout)){
      switch(match_index){
        case 0: //+IPD, ... read and buffer charaters until you get a colon
          ESP8266_AT_Client::DEBUG("Got '+IPD,'");
          if(readStreamUntil(":", (char *) tmp, 31, 10)){            
            ESP8266_AT_Client::DEBUG("Got ':'");          
            // response buffer tells us how many bytes we expect to receive in this chunk
            // we just need to convert it to an integer first
            char * ptr = NULL;
            uint32_t num_characters_expected = strtoul((char *) tmp, &ptr, 0);
            if (*ptr != '\0'){
               ESP8266_AT_Client::DEBUG("Receive Error, length parse error");         
               ret = 4; // failed to parse
            }
            else{
               ESP8266_AT_Client::DEBUG("Receive expect: ", num_characters_expected);         
            }
                                  
            if(readStream(num_characters_expected, 5000)){
               ret = 1; // expected number of characters was received
               // wait for "OK"
               if(readStreamUntil(target_match_array, &match_index, timeout)){
                  ESP8266_AT_Client::DEBUG("Receive complete");                                  
                  if(match_index == 2){ // OK  
                    ESP8266_AT_Client::DEBUG("Terminated by 'OK'");                                          
                  }
                  else if(match_index == 1){ // CLOSED
                    ESP8266_AT_Client::DEBUG("Terminated by 'CLOSED'");   
                  }                  
                  else{
                    ESP8266_AT_Client::DEBUG("Unexpected termination of ", match_index);   
                  }
               }
               else{
                  ESP8266_AT_Client::DEBUG("Timed out waiting for Receive terminator");   
               }
            }
            else{
              ret = 3; // timeout
            }          
          }
          else{         
             ESP8266_AT_Client::DEBUG("Receive Error, ':' missing");        
             ret = 4; // parse error
          }
          break;
        case 1:
           ESP8266_AT_Client::DEBUG("Receive Complete");       
           ret = 2; // connection closed by remote host
           break;
        default:
           ESP8266_AT_Client::DEBUG("Unexpected State");     
           ret = 4;
      }
    }
    else{


//      ESP8266_AT_Client::DEBUG("Receive Timeout");


       ret =  3; // timeout
    }
  }
  
  return ret;
}

void ESP8266_AT_Client::DEBUG(char * msg){
#ifdef ESP8266_AT_CLIENT_ENABLE_DEBUG       
  if(ESP8266_AT_Client::debugStream != NULL){
    ESP8266_AT_Client::debugStream->print("Debug: ");     
    ESP8266_AT_Client::debugStream->println(msg);
  }
#endif
}

void ESP8266_AT_Client::DEBUG(char * msg, uint32_t value){
#ifdef ESP8266_AT_CLIENT_ENABLE_DEBUG  
  if(ESP8266_AT_Client::debugStream != NULL){
    ESP8266_AT_Client::debugStream->print("Debug: ");     
    ESP8266_AT_Client::debugStream->print(msg);
    ESP8266_AT_Client::debugStream->println(value);
  }
#endif
}

void ESP8266_AT_Client::DEBUG(char * msg, char * value){
#ifdef ESP8266_AT_CLIENT_ENABLE_DEBUG  
  if(ESP8266_AT_Client::debugStream != NULL){
    ESP8266_AT_Client::debugStream->print("Debug: ");     
    ESP8266_AT_Client::debugStream->print(msg);
    ESP8266_AT_Client::debugStream->print("\"");
    ESP8266_AT_Client::debugStream->print(value);
    ESP8266_AT_Client::debugStream->println("\"");
  }
#endif
}
