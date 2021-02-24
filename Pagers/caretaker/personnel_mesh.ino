//NOTE: WORKING WITH ACTIVE LOW FOR BUTTONS

//------------Header Files------------//
#include<Adafruit_SSD1306.h>
#include<painlessMesh.h>
//#include <Arduino_JSON.h>
//------------OLED Definitions------------//
#define height 32
#define width 128
#define reset 4
Adafruit_SSD1306 display(width,height, &Wire, reset);
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
#define LED_BUILTIN 2
/*postCode defs :
 * 1 - OLED not able to initialize
 * 2 - reply could not be sent
*/

// User stub
//The Task object is required for sending messages, tried using standalone conditions in loop(),
//did not work - message queue becomes full instantly
void sendMessage() ; // Prototype so PlatformIO doesn't complain
Task taskSendMessage( TASK_SECOND * 1 , TASK_FOREVER, &sendMessage );
void sendMessage() {
  if(digitalRead(ping) == LOW)
  {
  String msg = "Additional help requested by ";
  msg += userName;
  mesh.sendBroadcast( msg );
  Serial.println(msg);
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Sent request!");
  Serial.println("Sent request!");
   display.display();
   delay(2000);
  }
  taskSendMessage.setInterval( random( TASK_SECOND * 1, TASK_SECOND * 1 ));
}


// Needed for painless library
void receivedCallback( uint32_t from, String &msg ) 
{
  display.clearDisplay();
  display.setCursor(0,0);
  display.printf(msg.c_str());
  display.display(); 
  Serial.println(msg.c_str());
  //checks if node is still connected
  if(!mesh.isConnected(from))
  { 
    postCode = 2;
    return;
    }
  //check if the received message is a request
  

 
    int state;
    for(int i = 0; i<1000; i++)
      { state = digitalRead(ping);
        if(digitalRead(ping)==LOW) //gives user time (10s) to respond to the request
        break;  //if so, breaks from the for loop
        delay(10);
      }
      
      if(state) //checking if button is pressed
      {
      display.clearDisplay();
      display.setCursor(0,16);
      display.println("\t EC Wheel"); //returning to title screen
      display.display();
      return;
      }

   // need to concat this message using the concat function, dont remember, too lazy to search for it :P
   String reply = "The request has been answered by ";
   reply += userName;
   //letting others know request has been asnwered
   mesh.sendBroadcast( reply );
   //letting the requesting personnel know their request has been answered
   reply = "The answered request from ";
   reply += userName;
   mesh.sendSingle(from,reply);
   Serial.println("reply sent!");
    display.clearDisplay();
    display.setCursor(0,16);
    display.println("reply sent!"); //returning to title screen
    display.display();
    delay(3000);
   
  
  
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



//------------Setup------------//
void setup() {
  Wire.begin();
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ping, INPUT);

  //checking for oled 
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("Didn't load display\n");
    postCode = 1;
    return;
  } 
  
  Serial.println("Loaded display\n");
  
  
  display.clearDisplay();
  display.display();
  
  //setting oled parameters
  
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,10);
  display.println("Booting up");
  display.display();
  display.setCursor(0,18);
  for(int i = 0; i<20; i++)
  {
    display.print("|");
    display.display();
    delay(100);
    }

  
  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages
  nodeId = mesh.getNodeId();
  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  
  //nodeId is only for debugging purposes and as a result not required to be displayed on screen
  Serial.print("Node ID: ");
  Serial.println(nodeId);

   
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  userScheduler.addTask( taskSendMessage );
  taskSendMessage.enable();
  
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
  display.clearDisplay();
  display.setCursor(20,16);
  display.println("\t EC Wheel \t");
  display.display();      
  mesh.update(); 
      }
}
