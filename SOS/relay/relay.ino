//------------Header Files------------//
#include<Adafruit_SSD1306.h>
#include<painlessMesh.h>
#include <Wire.h> 
//#include <Arduino_JSON.h>

//------------Mesh Definitions------------//
#define   MESH_PREFIX     "testMesh"
#define   MESH_PASSWORD   "password"
#define   MESH_PORT       5555
// to control your personal task
Scheduler userScheduler; 
painlessMesh  mesh;
uint32_t nodeId;
String userName = "User B";
//------------Misc Definitions------------//
#define ping 5
uint8_t postCode = 0;
/*postCode defs :
 * 1 - OLED not able to initialize
 * 2 - reply could not be sent
*/

// Needed for painless library
void receivedCallback( uint32_t from, String &msg ) { 
  Serial.println(msg.c_str());
}
void newConnectionCallback(uint32_t nodeId) {
    Serial.printf("New Connection, nodeId = %u\n", nodeId);
}
void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
}
void nodeTimeAdjustedCallback(int32_t offset) {
    Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),offset);
}

#define LED_BUILTIN 2

//------------Setup------------//
void setup() {
  Wire.begin();
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ping, INPUT);
  
  Serial.println("setup \n");
  
  //setting oled parameters

  
  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages
  nodeId = mesh.getNodeId();
  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  
  //nodeId is only for debugging purposes
  Serial.print("Node ID: ");
  Serial.println(nodeId);
   
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  
}

//------------Loop------------//
void loop() 
{
  //adding mechanism for detecting error code without use of serial monitor
  if(postCode)
  {
      Serial.print("ERROR! code: ");
      Serial.print(postCode);
      for(int i=0;i<postCode;i++)
      {
        digitalWrite(LED_BUILTIN,HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN,LOW);
        delay(500);
        }
        delay(2000);
       
      
      }
      else
      {
  mesh.update(); 
      }
}
