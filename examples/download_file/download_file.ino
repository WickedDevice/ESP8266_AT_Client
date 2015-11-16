#include <ESP8266_AT_Client.h>
#include <util/crc16.h>

#define NETWORK_SSID     "YOUR NETWORK NAME"
#define NETWORK_PASSWORD "YOUR NETWORK PASSWORD"

#define DOWNLOAD_FILE_LOOP      (true)
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

void downloadFile(char * hostname, uint16_t port, char * filename, void (*responseBodyProcessor)(uint8_t *, uint32_t));
void processResponseData(uint8_t * data, uint32_t data_length);

char scratch[1024] = {0};

void setup(void){
  Serial.begin(115200);               // debug console
  Serial1.begin(115200);              // AT command interface

  //ESP8266_AT_Client::debugStream = &Serial;
  
  esp.setInputBuffer(input_buffer, ESP8266_INPUT_BUFFER_SIZE); // connect the input buffer up
  esp.reset();                                                 // reset the module
  
  Serial.print("Set Mode to Station...");
  esp.setNetworkMode(1);
  Serial.println("OK");   
  
  Serial.print("Connecting to Network...");  
  esp.connectToNetwork(NETWORK_SSID, NETWORK_PASSWORD, 60000, NULL); // connect to the network
  Serial.println("OK");  
  
  do {
    downloadFile("update.wickeddevice.com", 80, "archive/aqev2_no2_co-2.0.5.hex", processResponseData);
  } 
  while(DOWNLOAD_FILE_LOOP);
}

void loop(void){
  if(Serial1.available()){
    Serial.write(Serial1.read());
  }

  if(Serial.available()){
    Serial1.write(Serial.read());
  }
}

uint16_t crc16_checksum = 0;
uint32_t body_bytes_received = 0;
boolean past_header = false; 

void downloadFile(char * hostname, uint16_t port, char * filename, void (*responseBodyProcessor)(uint8_t *, uint32_t)){      
  unsigned long total_bytes_read = 0;
  uint8_t mybuffer[64] = {0};
  
  // re-initialize the globals
  crc16_checksum = 0;
  body_bytes_received = 0;   
  past_header = false;  
  
  /* Try connecting to the website.
     Note: HTTP/1.1 protocol is used to keep the server from closing the connection before all data is read.
  */ 
  esp.connect(hostname, port);
  if (esp.connected()) {   
    memset(scratch, 0, 1024);
    snprintf(scratch, 1023, "GET /%s HTTP/1.1\r\nHost: %s\r\n\r\n\r\n", filename, hostname);        
    esp.print(scratch);
    //Serial.print(scratch);    
  } else {
    Serial.println(F("Error: Server Connection failed"));    
    return;
  }

  Serial.println(F("Info: -------------------------------------"));
  
  /* Read data until either the connection is closed, or the idle timeout is reached. */ 
  unsigned long lastRead = millis(); 
  unsigned long num_bytes_read = 0;  
  unsigned long start_time = millis();
  uint32_t loop_counter = 0;
  
  while (esp.connected(false) && (millis() - lastRead < IDLE_TIMEOUT_MS)) {        
    while (esp.available()) {
      //char c = esp.read();
      num_bytes_read = esp.read(mybuffer, 64);     
      total_bytes_read += num_bytes_read;
      
      loop_counter++;
      if((loop_counter % 4096) == 0){
        Serial.print(".");
      }

      if(responseBodyProcessor != 0){
        responseBodyProcessor(mybuffer, num_bytes_read); // signal end of stream
      }        
        
      lastRead = millis();
    }
  }

//  Serial.print("Remaining bytes: ");
//  Serial.println(esp.available());
   
  esp.stop();
  
  Serial.println();  
  Serial.println("Debug: Download Complete");
  Serial.print("Total Bytes: ");
  Serial.println(total_bytes_read);
  Serial.print("File Size: ");
  Serial.println(body_bytes_received);
  Serial.print("Checksum: ");
  Serial.println(crc16_checksum);  
  Serial.print("Duration: ");
  Serial.println(millis() - start_time);   
  
  
}

void processResponseData(uint8_t * data, uint32_t data_length){
  uint32_t start_index = 0;         
  static uint8_t header_guard_index = 0; 
  
  if(!past_header){
    for(uint32_t ii = 0; ii < data_length; ii++){                 
      switch(header_guard_index){
      case 0:
        if(data[ii] == '\r') header_guard_index++;
        break;
      case 1:
        if(data[ii] == '\n') header_guard_index++;
        else header_guard_index = 0;        
        break;
      case 2:
        if(data[ii] == '\r') header_guard_index++;
        else header_guard_index = 0;         
        break;
      case 3:
        if(data[ii] == '\n') header_guard_index++;
        else header_guard_index = 0;         
        break;
      case 4:        
        past_header = true;
        start_index = ii;
        header_guard_index = 0;
        break;
      }      
    }
  }

  if(past_header){
    body_bytes_received += data_length - start_index;
    for(uint32_t ii = start_index; ii < data_length; ii++){     
      crc16_checksum = _crc16_update(crc16_checksum, data[ii]);
    }
  }
    
}