/*

Created by Andre Rumantir, Origo, 2018

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

=================================================================


*/

#include <SoftwareSerial.h>
#include <dynaBus.h>

#if defined(__AVR_ATmega328P__)

SoftwareSerial meshbee(13,12); //(RX, TX)
#define debug Serial

#else if defined(__AVR_ATmega32U4__)

#define meshbee Serial1
#define debug Serial

#endif

//API Identifiers
#define API_DATA_PACKET 0x02
#define API_LOCAL_AT_REQ 0x08
#define API_LOCAL_AT_RESP 0x88

//AT Commands table
#define ATIF 0x54

#define START_DELIMITER 0x7E

#define INTERVAL 3000 //interval for measurements sent (ms)
#define ATIF_RETRY 5000 //delay to retry sending ATIF command if unsuccessful
#define MAX_PAYLOAD_SIZE 45

//SENSOR PINS
#define RAINFALL_PIN 2 //Rain bucket pin
#define OW_PIN 4 //One-Wire pin

/**
 * 
 * refer to http://wiki.seeedstudio.com/images/a/a5/MeshBee_User_Manual_v0.3.pdf
 * for API frame structure
 * 
 */
struct Node_Info{
  byte role;
  byte radio_channel;
  uint16_t firmware;
  uint16_t shrt_addr;
  uint16_t pan_id;
  uint32_t mac_low;
  uint32_t mac_high;
}node;

typedef struct Meshbee_API{
  byte delimiter=START_DELIMITER;
  byte payload_size;
  byte api_id;
  uint8_t payload[MAX_PAYLOAD_SIZE];
  byte checksum;
};

typedef struct Local_AT_Resp_Payload{
  byte frame_id;
  byte at_index;
  byte status;
  byte data_size;
  uint8_t data[20];  
};

typedef enum _status{
  COMPLETE = 1,
  RUNNING = 2,
  ERR = 3
};

byte b = 0, _pos = 0;
Meshbee_API recv = Meshbee_API(), apiPkt = Meshbee_API();

dynaBus ds(OW_PIN);
unsigned long lastmils = 0, lastatif = 0;
bool flag = 0;
volatile unsigned int rainfall_count = 0;
volatile unsigned long isrmils = 0;

void setup() {
  meshbee.begin(19200);
  debug.begin(19200);

  ds.begin();
  ds.find();

  //init sensor pins
  pinMode(OW_PIN,INPUT);
  
  pinMode(RAINFALL_PIN,INPUT);
  attachInterrupt(digitalPinToInterrupt(RAINFALL_PIN), rainfall_isr, FALLING);

  //making sure it is in correct mode
//  meshbee.write("+++"); //AT mode
//  delay(2000);
//  meshbee.write("ATAP"); //API mode
}

void loop() {
  unsigned long mils = millis();

  if (flag){
    debug.println("INTERRUPTED");
    flag = 0;
  }

  if (node.mac_low == 0 && node.mac_high == 0 && (mils - lastatif) >= ATIF_RETRY ){
    lastatif = mils;
    prepLocalATPkt(ATIF, &apiPkt);
    sendPkt(&apiPkt);
  }
  
  _status status = readPacket();
  if (status == COMPLETE || status == ERR){
    _pos = 0, b = 0, recv.checksum = 0; //reset

    if (status == COMPLETE){
      switch(recv.api_id){
        case API_LOCAL_AT_RESP:
          Local_AT_Resp_Payload atpy;
          dissectPayloadRes((uint8_t*)&atpy);
          atResponses(atpy);
          break;
      }
    }
  }

  if ((mils - lastmils) >= INTERVAL){
    lastmils = mils;
    char data[32] = {}; //max data packet body is 32bytes
    uint8_t dataSize = getData(data);
    
    prepDataPkt(&apiPkt, data, dataSize);
    sendPkt(&apiPkt);
  }
}


/**
 * 
 * =================================================
 * Functions in preparation of receiving API packets
 * =================================================
 * 
 */

/**
 * 
 * Description:
 * dissecting payload from response to parameter's structure
 * 
 */
void dissectPayloadRes(uint8_t *payload){
  for (int i=0;i< recv.payload_size;i++){
    *(payload + i) = recv.payload[i];
  }
}

/**
 * 
 * Description:
 * Setting AT response packet into object structure
 * 
 */
void atResponses(Local_AT_Resp_Payload atpy){
  switch (atpy.at_index){
    case ATIF: //store node's information
      node.role = atpy.data[0];
      node.radio_channel = atpy.data[1];
      node.firmware = (atpy.data[2] << 8) | (atpy.data[3]);
      node.shrt_addr = (atpy.data[4] << 8) | (atpy.data[5]);
      node.pan_id = (atpy.data[6] << 8) | (atpy.data[7]);
      node.mac_low = (uint32_t)atpy.data[8] << 24 | (uint32_t)atpy.data[9] << 16 | (uint32_t)atpy.data[10] << 8 | atpy.data[11];
      node.mac_high = (uint32_t)atpy.data[12] << 24 | (uint32_t)atpy.data[13] << 16 | (uint32_t)atpy.data[14] << 8 | atpy.data[15];
  }
}

/**
 * 
 * Description:
 * read API response on meshbee serial interface
 * 
 * @return none
 * 
 */
_status readPacket(){
  while (meshbee.available()){
    b = meshbee.read();

    //new packet before previous packet completed
    if (_pos > 0 && b == START_DELIMITER){
      debug.println(F("ERR"));
      return ERR;
    }
    
    switch(_pos){
      case 0: //delimiter
        if (b == START_DELIMITER){
          _pos++;
        }
        break;
      
      case 1: //length of payload
        recv.payload_size = b;
        _pos++;
        break;
      
      case 2: //api identifier
        recv.api_id = b;
        _pos++;
        break;
        
      default: //payload + checksum
        if (_pos > MAX_PAYLOAD_SIZE){
          debug.println(F("ERR"));
          return ERR;
        }

        //3 for delimiter, length, api id
        if (_pos < recv.payload_size+3){
          recv.payload[_pos-3] = b;
          _pos++; 
          recv.checksum += b;
        } else {
          if (recv.checksum != b){
            debug.println(F("CHECKSUM ERR"));
            return ERR;
          }

          return COMPLETE;
        }
        break;
    }
  }

  return RUNNING;
}


/**
 * ===============================================
 * Functions in preparation of sending API packets
 * ===============================================
 */

/**
 * 
 * description: 
 * prepare payload packet according to doc as API_DATA_PACKET
 * 
 * @return payload size
 * 
 */
void prepDataPkt(Meshbee_API *apiPkt, char* data, uint8_t dataSize){
  apiPkt->payload_size = dataSize + 15;
  apiPkt->api_id = API_DATA_PACKET;
  
  apiPkt->payload[0] = random(0,255); //frame ID
  apiPkt->payload[1] = 0x00; //option
  apiPkt->payload[2] = 0x00; //unicast addr high byte
  apiPkt->payload[3] = 0x00; //unitcast addr low byte
  apiPkt->payload[4] = 0x00; apiPkt->payload[5] = 0x00; apiPkt->payload[6] = 0x00; apiPkt->payload[7] = 0x00; apiPkt->payload[8] = 0x00; apiPkt->payload[9] = 0x00; apiPkt->payload[10] = 0x00; apiPkt->payload[11] = 0x00; //unicast long addr

  apiPkt->payload[12] = dataSize+2; //inclusive of \r \n
  for (int i=0;i<dataSize;i++){
    apiPkt->payload[13+i] = data[i];
  }

  //append end of line and carriage return for each data
  apiPkt->payload[13+dataSize] = '\r';
  apiPkt->payload[14+dataSize] = '\n';


  apiPkt->checksum = 0; //reset previous checksum
  for (int i=0; i < apiPkt->payload_size;i++){
    apiPkt->checksum += apiPkt->payload[i];
  }
}

/**
 * 
 * description: 
 * prepare local AT packet according to doc
 * 
 */
void prepLocalATPkt(uint8_t cmd, Meshbee_API *apiPkt){
  apiPkt->payload[0] = random(0,255); //frame ID
  apiPkt->payload[1] = 0x00; //option
  
  apiPkt->payload[2] = cmd;

  switch(cmd){
    case ATIF:
      for (int i=0;i<4;i++){
        apiPkt->payload[i+3] = 0x00;
      }
      break;
  }

  apiPkt->payload_size = 7;
  apiPkt->api_id = API_LOCAL_AT_REQ;

  apiPkt->checksum = 0;
  for (int i=0; i < apiPkt->payload_size;i++){
    apiPkt->checksum += apiPkt->payload[i];
  }
}

/**
 * 
 * Description:
 * transmit packet data
 * 
 */
void sendPkt(Meshbee_API *apiPkt) {
  meshbee.write(apiPkt->delimiter); //start delimiter
  meshbee.write(apiPkt->payload_size); //length
  meshbee.write(apiPkt->api_id); //API identifier
  meshbee.write(apiPkt->payload, apiPkt->payload_size); //API_DATA_PACKET frame
  meshbee.write(apiPkt->checksum); //check sum
}

/**
 * 
 * description: 
 * captures sensor readings and store them into assigned data buffer
 * 
 * data format:
 * MAC_ADDR,TEMP,RAINFALL,HUMIDITY
 * 
 * @param *data
 * 
 * @return data size length
 * 
 */
uint8_t getData(char *data){
  uint8_t data_size;
  char temp[6]="-999",humid[6]="-999";

  for (int i=0;i<ds.nb();i++){
    if(ds[i*8]==0x26){ //DS2438
      float vdd = ds.get26voltage(i,"vdd");
      float vad = ds.get26voltage(i,"vad");
      
      float t = ds.get26temperature(i);
      float h = (vad/vdd - 0.16)/0.0062;

      dtostrf(t,5,2,temp);
      dtostrf(h,5,2,humid);
    } else if(ds[i*8]==0x28) { ///DS18B20
      float t = ds.get28temperature(i);
      dtostrf(t,5,2,temp);
    }
  }
  
  data_size = sprintf(data,"%08lx%08lx,%s,%u,%s", node.mac_high,node.mac_low,temp,rainfall_count,humid);
  
  debug.println(data);

  rainfall_count = 0;

  return data_size;
}


/**
 * 
 * ================================
 * Interrupt Service Routines (ISR)
 * ================================
 * 
 */
void rainfall_isr(){

  unsigned long mils = millis();

  if ((mils - isrmils) > 100UL){
    isrmils = mils;
    rainfall_count++;
    flag = 1;
  }
}

