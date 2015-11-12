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
  uint8_t ret = 0;
  
  if(socket_connected){
    // set up an AT command and send it
    // then return whether or not it succeeded

    clearTargetMatchArray();
    addStringToTargetMatchList("STATUS:3");
    addStringToTargetMatchList("STATUS:4"); // disconnected
    //addStringToTargetMatchList("STATUS:2"); // got ip
    addStringToTargetMatchList("OK\r\n");  
    addStringToTargetMatchList("+IPD,");    
        
    // you have to get "STATUS:" and a numeric code
    // then you have *may* get "+CIPSTATUS:"
    // then you get "OK" 
    if(num_characters_remaining_to_receive > 0){
      receive(true); // receive is already in progress
      ret = 1;
    }
    else{    
      stream->print("AT+CIPSTATUS");
      stream->print("\r\n");  
        
      uint8_t match_index = 0xFF;      
      if(readStreamUntil(&match_index, 100)){      
        ret = 1;     
        if(match_index == 3){ // +IPD, gotta deal with it
          receive(true); // delegate to the receive function to handle it, but don't exit this 
        }   
        if(match_index == 1){ // STATUS:3 or STATUS:4
          socket_connected = false;
          ret = 0;    
        }
        else{
          // OK... shouldn't happen here under normal conditions
        }                              
      } 
      else{
        // no match within 100ms
        ret = 1; // but theoretically socket is still connected
      }          
      
      if(readStreamUntil(&match_index, 100)){ // we got a status code 
        if(match_index != 2){ //  != "OK"
           ESP8266_AT_Client::DEBUG("CIPSTATUS not OK");
           ret = 0;
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
      strncpy(tgt_addr, str, ESP8266_AT_CLIENT_MAX_STRING_LENGTH);  // copy the string in      
      return true;
    }
    
  }
  
  return false;
}

// pass in an array of strings to match against
// the list is presumed to be terminated by a NULL string
// this function can only handle up to ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES matching targets
boolean ESP8266_AT_Client::readStreamUntil(uint8_t * match_idx, char * target_buffer, uint16_t target_buffer_length, int32_t timeout_ms){
  boolean match_found = false;
  uint16_t target_buffer_index = 0;
  uint8_t match_char_idx[ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES] = {0};
  unsigned long previousMillis = millis();      
  
  ESP8266_AT_Client::DEBUG("+++");
  for(uint8_t ii = 0; ii < ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES; ii++){
    uint16_t target_match_length = strlen(target_match_array[ii]);
    if(target_match_length == 0){ 
      break;
    }

    ESP8266_AT_Client::DEBUG("  Waiting for ", target_match_array[ii]);  

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
      debugStream->print(chr); // echo the received characters to the Serial Monitor
#endif
      
      // for each target match
      for(uint8_t ii = 0; ii < ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES; ii++){
        uint16_t target_match_length = strlen(target_match_array[ii]);
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

boolean ESP8266_AT_Client::readStream(uint16_t num_characters_expected, int32_t timeout_ms){
  uint16_t num_characters_read = 0;
  unsigned long previousMillis = millis();   
  
#ifdef ESP8266_AT_CLIENT_DEBUG_ECHO_EVERYTHING   
  debugStream->print("readStream ");
  debugStream->println(num_characters_expected); // echo the received characters to the Serial Monitor
#endif  
  
  while(num_characters_read < num_characters_expected){
    unsigned long currentMillis = millis();    
    if((timeout_ms > 0) && (currentMillis - previousMillis >= timeout_ms)){
       return false; // timeout condition
    }
    
    uint8_t available = stream->available();
    if(available > 60){      
      PORTB |= _BV(4);    
      debugStream->print("  Hardware Buffer Overflow imminent ");
      debugStream->print(millis());
      debugStream->print(" ");
      debugStream->println(available);
    }    
    
    while(stream->available()){            
      previousMillis = millis(); // reset the timeout      
      char chr = stream->read();    
      PORTD ^= _BV(4);
               
#ifdef ESP8266_AT_CLIENT_DEBUG_ECHO_EVERYTHING   
      debugStream->print(chr); // echo the received characters to the Serial Monitor
#endif
      
      if(!writeToInputBuffer(chr)){
        ESP8266_AT_Client::DEBUG("Input Buffer Overflow!");
        return false; // buffer overflow!
      }
      num_characters_read++;
      
      if(num_characters_read == num_characters_expected){  
        return true; 
      }
    }    
  }
  
  //debugStream->print("Only received ");
  //debugStream->print(num_characters_read);
  //debugStream->print(" of ");
  //debugStream->print(num_characters_expected);
  //debugStream->println(" expected characters");
  return false;
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
    if(input_buffer_read_ptr == input_buffer_tail_ptr){
      input_buffer_read_ptr = input_buffer;      
    }
    else{
      input_buffer_read_ptr++;
    }
    
    num_consumed_bytes_in_input_buffer--;
    num_free_bytes_in_input_buffer++;
    return *input_buffer_read_ptr;
  }
  
  return -1;
}

void ESP8266_AT_Client::flushInput(){
  while(num_consumed_bytes_in_input_buffer > 0){ 
    char chr = stream->read();
    
#ifdef ESP8266_AT_CLIENT_DEBUG_ECHO_EVERYTHING    
    debugStream->println((uint8_t) chr, HEX); // echo the received characters to the Serial Monitor   
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

// returns 1 if input buffer is ready, 2 if connection closed, 3 if timeout, 4 on sync error
// the first three reasons (1, 2, 3) are kind of normal exits
// reason 3 (timeout) is kind of bad, and *might* be a good reason to force close the socket and start over
// the last reason (sync error) is really bad news - you should probably force close the socket and start over
uint8_t ESP8266_AT_Client::receive(boolean delegate_received_IPD){
  uint8_t ret = 0;
  const int32_t timeout = 10; // maximum time to wait for +IPD or CLOSED  

  if(num_free_bytes_in_input_buffer == 0){
    return 1;
  }
  
  if(stream->available() == 0){
    return 1;
  }

  //debugStream->print("BEGIN: Rx ");
  //debugStream->print(millis());
  //debugStream->print(" ");
  //debugStream->println(stream->available());

  boolean ongoing_processing = false;
  if(num_characters_remaining_to_receive > 0){
    ongoing_processing = true;
    //ESP8266_AT_Client::DEBUG("Receive continuation.");         
  }
  else{
    clearTargetMatchArray();
    addStringToTargetMatchList("+IPD,");
    addStringToTargetMatchList("CLOSED");  
  }
  
  //ESP8266_AT_Client::addStringToList(target_match_array, "OK", match_vector_length);
  while(ret == 0){
    char tmp[32] = {0};
    uint16_t num_characters_expected = 0;        
    uint8_t match_index = 0xFF;
    if(ongoing_processing || delegate_received_IPD || readStreamUntil(&match_index, timeout)){
            
      if(ongoing_processing){
        //debugStream->print("ongoing processing ");
        //debugStream->println(num_characters_remaining_to_receive);
        match_index = 0; // more bytes expected before the next +IPD,
      }
      if(delegate_received_IPD){
        //debugStream->print("delegate processing");
        //debugStream->println(num_characters_remaining_to_receive);
        match_index = 0; // i.e. it's as though you got +IPD
      }     
    
      switch(match_index){
        case 0: //+IPD, ... read and buffer charaters until you get a colon        
          if(!ongoing_processing){
            ESP8266_AT_Client::DEBUG("Got '+IPD,'");            
          }
                   
          if(ongoing_processing || readStreamUntil(":", (char *) tmp, 31, -1)){
            if(!ongoing_processing){
              ESP8266_AT_Client::DEBUG("Got ':'");
              // response buffer tells us how many bytes we expect to receive in this chunk
              // we just need to convert it to an integer first
              char * ptr = NULL;
              num_characters_expected = strtoul((char *) tmp, &ptr, 0);
              num_characters_remaining_to_receive = num_characters_expected;
              if (*ptr != '\0'){               
                 ESP8266_AT_Client::DEBUG("Receive Error, length parse error on input", tmp);         
                 //debugStream->print("Receive Error, length parse error on input '");
                 //debugStream->print(tmp);
                 //debugStream->print("'");
                 ret = 4; // failed to parse
              }
              else{
                 ESP8266_AT_Client::DEBUG("Receive expect: ", num_characters_remaining_to_receive);  
                 //debugStream->print("Expect: ");
                 //debugStream->print(num_characters_remaining_to_receive);
                 //debugStream->print("('");
                 //debugStream->print(tmp);
                 //debugStream->print("')");
                 //debugStream->println();                 
              }
            }
            
            if(num_free_bytes_in_input_buffer < num_characters_remaining_to_receive){
              num_characters_expected = num_free_bytes_in_input_buffer;
              // debugStream->print("   aSub-chunk: ");
              // debugStream->println(num_characters_expected);
            }
            else{
              num_characters_expected = num_characters_remaining_to_receive;
              // debugStream->print("   bSub-chunk: ");
              // debugStream->println(num_characters_expected);              
            }                                                                  
              
            ESP8266_AT_Client::DEBUG("Receiving: ", num_characters_expected);    
            if(readStream(num_characters_expected, 5000)){ // this only times out if *no characters* are received for 5 seconds            
               num_characters_remaining_to_receive -= num_characters_expected;                           
               ret = 1; // expected number of characters was received
            }
            else{      
              //debugStream->print("Didn't receive expected number of characters");         
              ret = 3; // timeout
            }          
          }
          else{         
             ESP8266_AT_Client::DEBUG("Receive Error, ':' missing or otherwise timeout");
             ret = 4; // parse error
          }
          break;
        case 1:
           ESP8266_AT_Client::DEBUG("Receive Complete - connection closed by remote host");  
           ret = 2; // connection closed by remote host
           break;
        default:
           ESP8266_AT_Client::DEBUG("Unexpected State");   
           ret = 4;
      }
    }
    else{
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

void ESP8266_AT_Client::DEBUG(char * msg, uint16_t value){
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

void ESP8266_AT_Client::clearTargetMatchArray(void){
  for(uint8_t ii = 0; ii < ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES; ii++){
    target_match_array[ii][0] = NULL;
  }
}
