#include <ESP8266_AT_Client.h>
#include <util/crc16.h>

#define NETWORK_SSID     "YOUR NETWORK NAME"
#define NETWORK_PASSWORD "YOUR NETWORK PASSWORD"

#define DOWNLOAD_FILE_LOOP      (true)
#define DOWNLOAD_FILE
#define IDLE_TIMEOUT_MS  10000     // Amount of time to wait (in milliseconds) with no data 
                                   // received before closing the connection.  If you know the server
                                   // you're accessing is quick to respond, you can reduce this value.

int esp8266_enable_pin = 23; // Arduino digital the pin that is used to reset/enable the ESP8266 module
Stream * at_command_interface = &Serial1;  // Serial1 is the 'stream' the AT command interface is on

ESP8266_AT_Client esp(esp8266_enable_pin, at_command_interface); // instantiate the client object

#define ESP8266_INPUT_BUFFER_SIZE (1500)
uint8_t input_buffer[ESP8266_INPUT_BUFFER_SIZE] = {0};     // sketch must instantiate a buffer to hold incoming data
                                      // 1500 bytes is way overkill for MQTT, but if you have it, may as well
                                      // make space for a whole TCP packet

uint16_t downloadFile(char * hostname, uint16_t port, char * filename, void (*responseBodyProcessor)(uint8_t, boolean, unsigned long, uint16_t));

char scratch[1024] = {0};

void setup(void){
  Serial.begin(115200);               // debug console
  Serial1.begin(115200);              // AT command interface
  
  esp.setInputBuffer(input_buffer, ESP8266_INPUT_BUFFER_SIZE); // connect the input buffer up
  esp.reset();                                                 // reset the module
  
  Serial.print("Set Network to Station...");
  esp.setNetworkMode(1);
  Serial.println("OK");   
  
  Serial.print("Connecting to Network...");  
  esp.connectToNetwork(NETWORK_SSID, NETWORK_PASSWORD, 60000, NULL); // connect to the network
  Serial.println("OK");  
  
  downloadFile("update.wickeddevice.com", 80, "aqev2_no2_co-2.0.2.hex", processUpdateHexBody);
}

void loop(void){
  
}

uint16_t downloadFile(char * hostname, uint16_t port, char * filename, void (*responseBodyProcessor)(uint8_t, boolean, unsigned long, uint16_t)){    
  uint16_t ret = 0;

  // re-initialize the globals
  unsigned long total_bytes_read = 0;
  unsigned long body_bytes_read = 0;
  uint16_t crc16_checksum = 0;
  uint8_t mybuffer[512] = {0};
  
  /* Try connecting to the website.
     Note: HTTP/1.1 protocol is used to keep the server from closing the connection before all data is read.
  */ 
  esp.connect(hostname, port);
  if (esp.connected()) {
    memset(scratch, 0, 1024);
    snprintf(scratch, 1023, "GET /%s HTTP/1.1\r\nHost: %s\r\n\r\n\r\n", filename, hostname);    
    esp.print(scratch);
  } else {
    Serial.println(F("Error: Server Connection failed"));    
    return 0;
  }

  Serial.println(F("Info: -------------------------------------"));
  
  /* Read data until either the connection is closed, or the idle timeout is reached. */ 
  unsigned long lastRead = millis();
  unsigned long num_chunks = 0;
  unsigned long num_bytes_read = 0;
  unsigned long num_header_bytes = 0;
  unsigned long start_time = millis();
  
  #define PARSING_WAITING_FOR_CR       0
  #define PARSING_WAITING_FOR_CRNL     1
  #define PARSING_WAITING_FOR_CRNLCR   2
  #define PARSING_WAITING_FOR_CRNLCRNL 3  
  #define PARSING_FOUND_CRNLCRNL       4
  uint8_t parsing_state = PARSING_WAITING_FOR_CR;
  // get past the response headers    
  while (esp.connected() && (millis() - lastRead < IDLE_TIMEOUT_MS)) {   
    while (esp.available()) {
      //char c = esp.read();
      num_bytes_read = esp.read(mybuffer, 255);
      num_chunks++;
      

      if((num_chunks % 5) == 0){
        Serial.print(".");
      }      
      
      for(uint32_t ii = 0 ; ii < num_bytes_read; ii++){
         if(parsing_state != PARSING_FOUND_CRNLCRNL){
           num_header_bytes++;
         }
         
         switch(parsing_state){
         case PARSING_WAITING_FOR_CR:
           if(mybuffer[ii] == '\r'){
             parsing_state = PARSING_WAITING_FOR_CRNL;
           }
           break;
         case PARSING_WAITING_FOR_CRNL:
           if(mybuffer[ii] == '\n'){
             parsing_state = PARSING_WAITING_FOR_CRNLCR;
           }         
           else{
             parsing_state = PARSING_WAITING_FOR_CR;
           }
           break;
         case PARSING_WAITING_FOR_CRNLCR:
           if(mybuffer[ii] == '\r'){
             parsing_state = PARSING_WAITING_FOR_CRNLCRNL;
           }         
           else{
             parsing_state = PARSING_WAITING_FOR_CR;
           }         
           break;
         case PARSING_WAITING_FOR_CRNLCRNL:
           if(mybuffer[ii] == '\n'){
             parsing_state = PARSING_FOUND_CRNLCRNL;
           }         
           else{
             parsing_state = PARSING_WAITING_FOR_CR;
           }         
           break;             
         default:           
           crc16_checksum = _crc16_update(crc16_checksum, mybuffer[ii]);
           if(responseBodyProcessor != 0){
             responseBodyProcessor(mybuffer[ii], false, body_bytes_read, crc16_checksum);
             body_bytes_read++;
           }
           break;
         }               
      }
      //Serial.println(num_bytes_read);
      total_bytes_read += num_bytes_read;
      uint16_t address = 0;
      uint8_t data_byte = 0;       
      lastRead = millis();
    }
  }
  
  www.stop();
  
  if(responseBodyProcessor != 0){
    responseBodyProcessor(0, true, body_bytes_read, crc16_checksum); // signal end of stream
  }  
  
  unsigned long end_time = millis();
  Serial.println(F("Info: -------------------------------------"));
  Serial.print("Info: # Bytes Read: ");
  Serial.println(total_bytes_read);
  Serial.print("Info: # Chunks Read: ");
  Serial.println(num_chunks);
  Serial.print("Info: File Size: ");
  Serial.println(total_bytes_read - num_header_bytes);
  Serial.print("Info: CRC16 Checksum: ");
  Serial.println(crc16_checksum);
  Serial.print("Info: Download Time: ");
  Serial.println(end_time - start_time); 
  
  return num_header_bytes;
}

void processUpdateHexBody(uint8_t dataByte, boolean end_of_stream, unsigned long body_bytes_read, uint16_t crc16_checksum){
  static uint8_t page[256] = {0};
  static uint16_t page_idx = 0;
  static uint32_t page_address = 0;
  
  if(page_idx < 256){
    page[page_idx++] = dataByte;  
    if(page_idx >= 256){
       page_idx = 0;
    }
  }
  
  if(end_of_stream || (page_idx == 0)){
    if((page_address % 4096) == 0){
      //while(flash.busy()){;}    
      //flash.blockErase4K(page_address); 
      //while(flash.busy()){;}   
    }    
    
    uint16_t top_bound = 256;
    if(page_idx != 0){
      top_bound = page_idx;
    }
    //flash.writeBytes(page_address, page, top_bound);
    
    
    // clear the page
    memset(page, 0, 256);
    
    // advance the page address
    page_address += 256;
    
  }
  
  if(end_of_stream){ 
    Serial.print(F("Checksum: "));
    Serial.println(crc16_checksum);
    Serial.print(F("Filesize: "));
    Serial.println(body_bytes_read);
  }
}