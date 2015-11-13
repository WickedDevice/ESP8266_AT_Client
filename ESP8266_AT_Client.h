#ifndef ___ESP8266_AT_CLIENT_H___
#define ___ESP8266_AT_CLIENT_H___

#include <Arduino.h>
#include <Client.h>

#define ESP8266_AT_CLIENT_MAX_STRING_LENGTH      (32)
#define ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES (5)

class ESP8266_AT_Client : public Client {

public:
  static Stream * debugStream; // where Debug messages go

  ESP8266_AT_Client(uint8_t enable_pin);
  ESP8266_AT_Client(uint8_t enable_pin, Stream * stream);
  ESP8266_AT_Client(uint8_t enable_pin, Stream * stream, uint8_t * buf, uint16_t buf_length);
  
  void setStream(Stream * stream);
  void setInputBuffer(uint8_t * buf, uint16_t buf_length);
  
  boolean setNetworkMode(uint8_t mode);
  boolean connectToNetwork(char * ssid, char * pwd, int32_t timeout_ms, void (*onConnect)(void));
  boolean disconnectFromNetwork(void);
  boolean reset(void);
  
  
  int connect(IPAddress ip, uint16_t port);
  int connect(const char *host, uint16_t port);
  int connect(IPAddress ip);
  int connect(const char *host);
  
  size_t write(uint8_t);
  size_t write(const uint8_t *buf, size_t size);
  int available();
  int read();
  int read(uint8_t *buf, size_t sz);
  int peek();
  void flush();
  void stop();
  uint8_t connected();
  uint8_t connected(boolean actively_check);
  uint8_t connectedToNetwork();
  operator bool();
  
  boolean addStringToTargetMatchList(char * str);
private:
  Stream * stream;      // where AT commands are sent and responses received   
  boolean socket_connected;
  uint8_t enable_pin;
  uint8_t * input_buffer;
  uint16_t input_buffer_length;
  uint8_t * input_buffer_read_ptr;
  uint8_t * input_buffer_write_ptr;
  uint8_t * input_buffer_tail_ptr;  
  uint16_t num_consumed_bytes_in_input_buffer;
  uint16_t num_free_bytes_in_input_buffer;  
  
  uint16_t num_characters_remaining_to_receive;
  char target_match_array[ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES + 1][ESP8266_AT_CLIENT_MAX_STRING_LENGTH + 1]; 
  char target_match_lengths[ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES + 1];
  
  void clearTargetMatchArray(void);
  boolean writeToInputBuffer(uint8_t c);
  uint8_t readFromInputBuffer(void);
  
  boolean readStreamUntil(uint8_t * match_idx, char * target_buffer, uint16_t target_buffer_length, int32_t timeout_ms);
  boolean readStreamUntil(uint8_t * match_idx, int32_t timeout_ms);
  boolean readStreamUntil(uint8_t * match_idx);
    
  boolean readStreamUntil(char * target_match, char * target_buffer, uint16_t target_buffer_length, int32_t timeout_ms);        
  boolean readStreamUntil(char * target_match, int32_t timeout_ms);
  boolean readStreamUntil(char * target_match);

  void flushInput();
  void receive(boolean delegate_received_IPD = 0);
  static void DEBUG(char * msg);
  static void DEBUG(char * msg, uint16_t value);
  static void DEBUG(char * msg, char * value); 
};

#endif