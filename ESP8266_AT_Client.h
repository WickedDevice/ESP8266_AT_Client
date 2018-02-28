#ifndef ___ESP8266_AT_CLIENT_H___
#define ___ESP8266_AT_CLIENT_H___

#include <Arduino.h>
#include <Client.h>
#include <Stream.h>
#include <Print.h>

#define ESP8266_AT_CLIENT_MAX_STRING_LENGTH      (32)
#define ESP8266_AT_CLIENT_MAX_NUM_TARGET_MATCHES (5)

typedef struct {
  uint8_t security;
  char ssid[33];
  int8_t rssi;
  uint8_t mac[6];
} ap_scan_result_t;

typedef enum {
  ESP8266_TCP,
  ESP8266_UDP
} esp8266_connect_proto_t;

class ESP8266_AT_Client : public Client {

public:
  ESP8266_AT_Client(uint8_t enable_pin);
  ESP8266_AT_Client(uint8_t enable_pin, Stream * stream);
  ESP8266_AT_Client(uint8_t enable_pin, Stream * stream, uint8_t * buf, uint16_t buf_length);

  void setStream(Stream * stream);
  void setInputBuffer(uint8_t * buf, uint16_t buf_length);

  boolean setNetworkMode(uint8_t mode);
  boolean connectToNetwork(char * ssid, char * pwd, int32_t timeout_ms = 60000, void (*onConnect)(void) = NULL, boolean permanent = false);
  boolean disconnectFromNetwork(void);
  boolean reset(void);

  int connect(IPAddress ip, uint16_t port, esp8266_connect_proto_t proto);
  int connect(uint32_t ip, uint16_t port, esp8266_connect_proto_t proto);
  int connect(const char *host, uint16_t port, esp8266_connect_proto_t proto);
  int connect(IPAddress ip);
  int connect(const char *host);

  int connect(IPAddress ip, uint16_t port);
  int connect(const char *host, uint16_t port);

  boolean connectUDP(uint32_t ip, uint16_t port);
  boolean connectUDP(const char *host, uint16_t port);

  boolean getIPAddress(uint32_t * ip, uint32_t * gateway, uint32_t * netmask);
  boolean getMacAddress(uint8_t * mac);
  boolean getIPAddress(char * ip_str, char * gateway_str, char * netmask_str);
  boolean getMacAddress(char * mac_str);

  boolean getRemoteIp(uint32_t * ip);
  boolean getHostByName(const char *hostname, uint32_t *ip, uint32_t timeout_ms = 5000);
  boolean scanAccessPoints(ap_scan_result_t * results, uint8_t max_num_results, uint8_t * num_results_found, uint32_t timeout_ms = 10000);
  boolean scanForAccessPoint(char * ssid, ap_scan_result_t * result, uint8_t * num_results_found, uint32_t timeout_ms = 10000);
  boolean setMacAddress(uint8_t * mac_address);
  boolean setStaticIPAddress(uint32_t ipAddress, uint32_t netMask, uint32_t defaultGateway, uint32_t dnsServer);
  boolean setDHCP(void);
  boolean sleep(uint8_t mode);
  boolean setTcpKeepAliveInterval(uint16_t _tcp_seconds); // 0 - 7200; 0 is disabled
  boolean configureSoftAP(const char *ssid, const char * pwd, uint8_t channel, uint8_t security);

  // utility functions
  uint32_t IpArrayToIpUint32(uint8_t * ip);
  void IpArrayToString(uint8_t * ip, char * tgt);
  void IpUint32ToArray(uint32_t ip, uint8_t * ip_arr);
  void IpUint32ToString(uint32_t ip, char * tgt);
  boolean stringToIpUint32(char * str, uint32_t * ip);
  boolean stringToIpArray(char * str, uint8_t * ip);
  void enableDebug(void);
  void disableDebug(void);
  void setDebugStream(Stream * ds);

  void macArrayToString(uint8_t * mac, char * tgt);
  boolean stringToMacArray(char * str, uint8_t * mac);

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
  boolean listen(uint16_t port);

  boolean firmwareUpdateBegin();
  boolean firmwareUpdateStatus(uint8_t * status);
  boolean getVersion(char * version);
  boolean getVersion(uint32_t * version);
  boolean restoreDefault();

  boolean AT(void);

  operator bool();

  boolean addStringToTargetMatchList(char * str);
private:
  Stream * stream;       // where AT commands are sent and responses received
  Print * streamAsPrint; // in order to check for available space in output buffer
  Stream * debugStream; // where Debug messages go
  boolean debugEnabled;

  boolean socket_connected;
  boolean wifi_is_connected;
  boolean listener_started;
  boolean ok_flag;
  boolean ready_flag;
  boolean error_flag;

  esp8266_connect_proto_t socket_type;
  uint16_t tcp_keep_alive_interval_seconds;

  uint8_t enable_pin;
  uint8_t * input_buffer;
  uint16_t input_buffer_length;
  uint16_t input_buffer_read_idx;
  uint16_t input_buffer_write_idx;

  uint16_t num_consumed_bytes_in_input_buffer;
  uint16_t num_free_bytes_in_input_buffer;
  
  void processIncomingUpToColon(void);   // should be called anytime right after you see +IPD,
  boolean processIncomingAfterColon(void);  // should be called frequently (as in non-blocking implementation), while incoming data is pending
  int32_t numIncomingBytesPending = 0;  // this should be counted down to zero by processIncomingAfterColon  
  boolean updatePlusIpdState(uint8_t c); // anytime a character is received this should be called with that caracter as an argument 
                                         // returns true if handleIncoming should be called
  int16_t streamReadChar(void);          // should only be called if it is known that there are bytes available
                                         // returns (int16_t) -1 if processing was interrupted by +IPD handling
  void waitForIncomingDataToComplete(void); // just calls processIncomingAfterColon in a spin loop, with timeout
  
  size_t streamWrite(const char * str);
  size_t streamWrite(uint32_t value);
  size_t streamWrite(int32_t value);
  size_t streamWrite(const uint8_t *buf, size_t sz); // write a buffer of known size to the stream
  
  boolean writeToInputBuffer(uint8_t c);
  int16_t readFromInputBuffer(void);
  void parseScanResult(ap_scan_result_t * result, char * line);

  void flushInput();
  void ESP8266_DEBUG(char * msg);
  void ESP8266_DEBUG(char * msg, uint16_t value);
  void ESP8266_DEBUG(char * msg, char * value);
  
  static uint8_t* debug_read_window;
  static uint16_t debug_read_window_index;
  static uint8_t* debug_write_window;
  static uint16_t debug_write_window_index;
  static void printDebugWindow(void);
  static void addToDebugReadWindow(uint8_t b);
  static void addToDebugWriteWindow(uint8_t b);
  static uint16_t bytesAvailableMax;
};

#endif
