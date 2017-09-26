#include <ESP8266_AT_Client.h>
#include <TinyWatchdog.h>
#include <stdint.h>

#define NETWORK_SSID     "YOUR NETWORK NAME"
#define NETWORK_PASSWORD "YOUR NETWORK PASSWORD"

int esp8266_enable_pin = 23; // Arduino digital the pin that is used to reset/enable the ESP8266 module
Stream * at_command_interface = &Serial1;  // Serial1 is the 'stream' the AT command interface is on
ESP8266_AT_Client esp(esp8266_enable_pin, at_command_interface); // instantiate the client object
TinyWatchdog tinywdt;
unsigned long current_millis = 0;

#define ESP8266_INPUT_BUFFER_SIZE (1500)
uint8_t input_buffer[ESP8266_INPUT_BUFFER_SIZE] = {0};     // sketch must instantiate a buffer to hold incoming data
                                      // 1500 bytes is way overkill for MQTT, but if you have it, may as well
                                      // make space for a whole TCP packet

void setup(void){
  Serial.begin(115200);               // debug console
  Serial1.begin(115200);              // AT command interface
  tinywdt.begin(100, 65000);

  //ESP8266_AT_Client::debugStream = &Serial;
  esp.setInputBuffer(input_buffer, ESP8266_INPUT_BUFFER_SIZE); // connect the input buffer up
  esp.reset();                                                 // reset the module

  Serial.print("Set Mode to Station...");
  esp.setNetworkMode(1);
  Serial.println("OK");

  Serial.print("Connecting to Network...");
  esp.connectToNetwork(NETWORK_SSID, NETWORK_PASSWORD, 60000, NULL); // connect to the network
  Serial.println("OK");

  if(!esp.setDHCP()){
    Serial.println("Failed to set DHCP");
  }
  else{
    uint32_t ip = 0, gateway = 0, netmask = 0;
    if(!esp.getIPAddress(&ip, &gateway, &netmask)){
      Serial.println("Failed to get IP address");
    }
    else{
      char ip_str[16] = {0}, gateway_str[16] = {0}, netmask_str[16] = {0};
      esp.IpUint32ToString(ip, (char *) &(ip_str[0]));
      esp.IpUint32ToString(gateway, (char *) &(gateway_str[0]));
      esp.IpUint32ToString(netmask, (char *) &(netmask_str[0]));
      Serial.print("IP: ");
      Serial.println((char *) ip_str);
      Serial.print("Gateway: ");
      Serial.println((char *) gateway_str);
      Serial.print("Netmask: ");
      Serial.println((char *) netmask_str);
    }
  }

  esp.setInputBuffer(input_buffer, ESP8266_INPUT_BUFFER_SIZE); // connect the input buffer up
  doUpdate();
}

void loop(void){
  delay(200);
  tinywdt.pet();

  if(Serial1.available()){
    Serial.write(Serial1.read());
  }

  if(Serial.available()){
    Serial1.write(Serial.read());
  }
}

void doUpdate(){
   boolean timeout = false;
   boolean gotOK = false;

   int32_t timeout_interval =  300000; // 5 minutes

   Serial.print(F("Info: Starting ESP8266 Firmware Update..."));
   if(esp.firmwareUpdateBegin()){
     Serial.println(F("OK"));
     uint8_t status = 0xff;
     uint32_t previousMillis = millis();
     while(esp.firmwareUpdateStatus(&status) && !timeout){
       // Serial.print(F("Info: Pet watchdog @ "));
       // Serial.println(millis());
       tinywdt.pet();

       if(status == 2){
         gotOK = true;
         // Serial.println(F("ESP8266 returned OK."));
       }
       else if(status == 3){
         if(gotOK){
           Serial.println(F("Info: Firmware update completed successfully."));
           break; // break out of the loop and perform a reboot
         }
         else{
           //
           Serial.println(F("Warning: ESP8266 Reset occurred before receiving OK."));
           return;
         }
       }

       if(status != 0xff){
         previousMillis =  current_millis; // reset timeout on state change
         // Serial.print("ESP8266 Status Changed to ");
         // Serial.println(status);
         status = 0xff;
       }

       current_millis = millis();
       if(current_millis - previousMillis >= timeout_interval){
          timeout = true;
       }
     }
     if(timeout){
       Serial.print(F("Error: Firmware Update Timed Out."));
       return;
     }
     else if(status == 1){
       Serial.print(F("Error: ESP8266 returned ERROR."));
       return;
     }
   }
   else {
     Serial.println(F("Failed."));
     return;
   }

   Serial.print(F("ESP8266 restoring defaults..."));
   if(esp.restoreDefault()){
     Serial.println(F("OK."));
   }
   else{
     Serial.println(F("Failed."));
   }

   Serial.println(F("Perform hardware reset here."));
}
