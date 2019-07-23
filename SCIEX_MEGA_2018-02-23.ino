// TC 2/23/2018
// added F and E commands from Mega to enable and disable load button (when Full)
//MN 7/15 5:00

const bool Open=0;
const bool Close=1;
const bool Up=0;
const bool Down=1;

const int XYHome=0;
const int XYLoad=1;
const int XYRackEmpty=2;
const int XYRackTrash=3;
const int XYTrash=4;

const int Empty=0;
const int Ready=1;
const int Sampling=2;
const int Pending=3;
const int Trash=4;


const int NoTrash_CON 	= -1;
bool Full = 0;
bool Home=0;
bool Vial=0;

int NextEmpty=1;
int NextTrash=0;
int rack;

int biDimArray [50] //initialize to zero
  {0, 0, 0, 0, 0,    
   0, 0, 0, 0, 0,   
   0, 0, 0, 0, 0,   
   0, 0, 0, 0, 0,   
   0, 0, 0, 0, 0,   
   0, 0, 0, 0, 0,   
   0, 0, 0, 0, 0,   
   0, 0, 0, 0, 0,   
   0, 0, 0, 0, 0,   
   0, 0, 0, 0, 0};

long XLoc [50] //initialize to zero
  {32800, 36225, 39650, 43075, 46500,    
    32800, 36225, 39650, 43075, 46500,    
    32800, 36225, 39650, 43075, 46500,    
    32800, 36225, 39650, 43075, 46500,    
    32800, 36225, 39650, 43075, 46500,    
    32800, 36225, 39650, 43075, 46500,    
    32800, 36225, 39650, 43075, 46500,    
    32800, 36225, 39650, 43075, 46500,    
    32800, 36225, 39650, 43075, 46500,    
    32800, 36225, 39650, 43075, 46500};

long YLoc [50] //initialize to zero
  { 47300, 47300, 47300, 47300, 47300,
    43511, 43511, 43511, 43511, 43511,
    39722, 39722, 39722, 39722, 39722,
    35933, 35933, 35933, 35933, 35933, 
    32144, 32144, 32144, 32144, 32144,
    28356, 28356, 28356, 28356, 28356,
    24567, 24567, 24567, 24567, 24567,
    20778, 20778, 20778, 20778, 20778, 
    16989, 16989, 16989, 16989, 16989,
    13200, 13200, 13200, 13200, 13200};
    
int LastPos=0; 
int incomingByte = 0;


long XPOS = 0;
long YPOS = 0;
int ZPOS = 0;

int i = 0;

//PINOUTS
const int GripperPin=4;

//Z axis
const int Zaxis_Enable = 37; //Robot Theta Xaxis_Enable
const int Zaxis_A = 35; //Robot Theta A (Direction)
const int Zaxis_B = 33; //Robot Theta A (Direction)
const int Zaxis_HLFB = 31; //Robot Theta Xaxis_HLFB

//X axis
const int Xaxis_Enable = 29; //Robot Theta Xaxis_Enable
const int Xaxis_A = 27; //Robot Theta A (Direction)
//25 set to Robot Theta B below (pulse burst positioning)
const int Xaxis_HLFB = 23; //Robot Theta Xaxis_HLFB

//Y axis
const int Yaxis_Enable = 28; //Robot Theta Xaxis_Enable
const int Yaxis_A = 26; //Robot Theta A (Direction)
//24 set to Robot Theta B below (pulse burst positioning)
const int Yaxis_HLFB = 22; //Robot Theta Xaxis_HLFB

int CHFlag=6; //Yellow to  pin 13 Green/WHT on D-Sub 15 connector - TTLIn3  
int CTCFlag=9; //White - to pin 2 Brown on D-Sub 15 connector - TTLOut3

//DECLARE PINOUT STATES

void setup() {

  
  pinMode(CHFlag, OUTPUT);
  digitalWrite(CHFlag, HIGH);

  pinMode(CTCFlag, INPUT_PULLUP);
  
  Serial.begin(115200);
  Serial1.begin(38400);
  Serial.write("I1");
  Serial1.write("I");
  
  DDRA |= _BV (PA3); // low level command for pinMode (25, OUTPUT); X
  DDRA |= _BV (PA2); // low level command for pinMode (24, OUTPUT); Y

  //define for Z axis motor
  pinMode(Zaxis_Enable, OUTPUT);
  digitalWrite(Zaxis_Enable, LOW);
  pinMode(Zaxis_A, OUTPUT);
  digitalWrite(Zaxis_A, LOW);
  pinMode(Zaxis_B, OUTPUT);
  digitalWrite(Zaxis_B, LOW);
  pinMode(Zaxis_HLFB, INPUT);
  digitalWrite(Zaxis_HLFB, HIGH);

  //define for Y axis motor
  pinMode(Yaxis_Enable, OUTPUT);
  digitalWrite(Yaxis_Enable, LOW);
  pinMode(Yaxis_A, OUTPUT);
  digitalWrite(Yaxis_A, LOW);
  pinMode(Yaxis_HLFB, INPUT);
  digitalWrite(Yaxis_HLFB, HIGH);
  
  //define for X Axis Motor
  pinMode(Xaxis_Enable, OUTPUT);
  digitalWrite(Xaxis_Enable, LOW);
  pinMode(Xaxis_A, OUTPUT);
  digitalWrite(Xaxis_A, LOW);
  pinMode(Xaxis_HLFB, INPUT);
  digitalWrite(Xaxis_HLFB, HIGH);

  #define NOP __asm__ __volatile__ ("nop\n\t")
  #define NOM __asm__ __volatile__ ("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t")

  pinMode(GripperPin, OUTPUT);
  digitalWrite(GripperPin, LOW);



  PrintArray();


   digitalWrite(Zaxis_Enable, HIGH);
  Z_WaitForMove();
   digitalWrite(Yaxis_Enable, HIGH);
  Y_WaitForMove();
   digitalWrite(Xaxis_Enable, HIGH);
  X_WaitForMove();

 MoveTo(XYHome);  
 digitalWrite(CHFlag, LOW);
 Gripper(Open);
 Serial1.write("I");
   Serial.write("I2");

 
}

void loop() {
if (Serial.available() > 0){
      incomingByte = Serial.read();
      Serial.println("Serial USER Data"); 
     // Serial.println(incomingByte, DEC); 
}
 
if (Serial1.available() > 0){
      incomingByte = Serial1.read();
}

if (incomingByte==49){ //49= ascii 1
  PrintArray();
  incomingByte=0;
  Vial=HIGH;
  Serial.println("Load Button");
}

if (incomingByte>64){ //65= ascii A...
  AssignTrash(incomingByte);
  PrintArray();
  incomingByte=0;
  Serial.println("Trash Button");
}

  CheckArray();
  
  if (Vial==HIGH){
    rack = PlaceInNextRow();
    if (rack > 50){
      Serial.print("Rack Full"); 
      CheckTrash();
      }
    else {
      Serial1.write("L");
      Serial1.write(rack);
      Vial=LOW;      //ADDED 7/14/2017 920AM
      PickUpVial();
    }
  }
  if( Full != CheckFull()){
    if(Full){
      // was Full
      Full = false;
      HMI_Serial1.write("E");
      DEBUG_Serial.print("Rack Not Full");
    }
    else {
      Full = true;
      HMI_Serial1.write("E");
      DEBUG_Serial.print("Rack Full");
    }
  }

  if (Serial1.available()== 0){ //added 7/14
      CheckTrash(); //else 7/14
  }
}

//interupts, set vial, status of locations

void PickUpVial(){
 Serial.println("PickUpVial");
 MoveTo(XYLoad);
 Gripper(Open);
 Z(Down);
 Gripper(Close);
 Z(Up);
 while (digitalRead(CTCFlag)==1){
    if (Home==0){
      MoveTo(XYHome);
      Home=1;
    Serial.println("CTC FLAG HIGH");
    }
 }
 Home=0;
 PlaceVial();
 Serial.println("Vial Placed");
}

void CheckTrash(){
  if (NextTrash!=-1){
    if (digitalRead(CTCFlag)==1){
      MoveTo(XYHome);
      return;
    }
    else TrashVial();
  }
  else {
    MoveTo(XYHome);
  }
}

void CheckArray(){
  for (int a = 0; a < 50; a++) {
      if (biDimArray [a] ==  4){
        NextTrash=a;
        return;
      }
      else NextTrash=-1;
  }
}

void PlaceVial(){
 Serial.println("PlaceVial");
  digitalWrite (CHFlag, HIGH);
  Vial=LOW;
  while (digitalRead(CTCFlag)==1) {}
  MoveTo(XYRackEmpty);
  Z(Down);
  Gripper(Open);
  StatusChange(Ready);
  Z(Up);
}

void TrashVial(){
  Serial.println("TrashVial");  
  Gripper(Open);
  digitalWrite (CHFlag, HIGH);
  while (digitalRead(CTCFlag)==1){}
  MoveTo(XYRackTrash);
  Z(Down);
  Gripper(Close);
  Z(Up);
  MoveTo(XYTrash);
  Z(Down);
  Gripper(Open);
  StatusChange(Empty);
  Z(Up);
}

void Gripper (bool State){
  if (State==HIGH){ digitalWrite(GripperPin, HIGH); delay(400);}
  else {digitalWrite(GripperPin, LOW); delay(300);}
}

void Z(bool State){
  if (State==LOW) {
    digitalWrite(Zaxis_A, LOW);    
    digitalWrite(Zaxis_B, LOW);
  }
  else {
    digitalWrite(Zaxis_A, HIGH);    
    digitalWrite(Zaxis_B, LOW);
  }
  Z_WaitForMove();
}

void MoveTo (int Location) {
  Z(Up);
  switch (Location) {
  case XYHome:
  MoveXY(0,13000);
  digitalWrite (CHFlag, LOW);
  break;
  
  case XYLoad:
  Serial.println("XYLoad");  
  MoveXY(0,0);
  digitalWrite (CHFlag, LOW);
  break;  
  
  case XYRackEmpty:
  Serial.println("XYRackEmpty"); 
  digitalWrite (CHFlag, HIGH);
  while (digitalRead(CTCFlag)==1){}
  MoveXY(XLoc[rack], YLoc[rack]);
  break;  
  
  case XYRackTrash:
  Serial.println("XYRackTrash"); 
  digitalWrite (CHFlag, HIGH);
  while (digitalRead(CTCFlag)==1) {}
  MoveXY(XLoc[NextTrash], YLoc[NextTrash]);
  break;
  
  case XYTrash:
  Serial.println("XYTrash"); 
  MoveXY(0,24000);
  digitalWrite (CHFlag, LOW);
  break;
  return;                   //if move to home, Trash, or Pickup digitalWrite (CHFlag, LOW);
  }
}

void StatusChange(int State){
  switch (State) {
  case Empty:
  biDimArray [NextTrash] = 0;
  Serial.print(NextTrash);
  Serial1.write("T");
  Serial1.write(NextTrash);
  NextTrash=-1;
  break;
  case Ready:
  break;  
  case Sampling:
  break;  
  case Pending:
  break;
  case Trash:
  break;
  return;                   //if move to home, Trash, or Pickup digitalWrite (CHFlag, LOW);
  }
}

///FILL NEXT 
int PlaceInNextRow(){
  Serial.println("PlaceInNextRow"); 
  int a;
    for (a = LastPos; a < 50; a++) {
        if (biDimArray [a] ==  0){
          biDimArray [a] =  1;
          LastPos=a;
          return LastPos;
        }
    }
    for (a = 0; a < 50; a++) {
        if (biDimArray [a] ==  0){
          biDimArray [a] =  1;
          LastPos=a;
          return LastPos;
        }
    }
    return 99;
}

void AssignTrash(int ASCII){
  Serial.println("AssignTrash"); 
  int b = ASCII-65;
  //if (biDimArray [b] !=  0){
        biDimArray [b] =  4;
  //}
        return;
 }

void PrintArray(){
  int a;
    for (a = 0; a < 50; a++) {
        Serial.print(biDimArray [a]);
        Serial.print(' ');
      }
      Serial.println();
    Serial.print("                   ");
      Serial.println(LastPos);
}



//MOTORS By=====> Locatoin (0, 0);    //X, Y
void MoveXY (long X, long Y)
{

  bool XDir = 0;
  if (X - XPOS < 0) XDir = 1;
  bool YDir = 0;
  if (Y - YPOS < 0) YDir = 1;


  //X and Y location
  XY_Move( abs(X - XPOS), XDir, abs(Y - YPOS), YDir);
  XPOS = X;
  YPOS = Y;
}


void XY_Move (long Xpulses, bool XDir, long Ypulses, bool YDir)
{

  digitalWrite(Xaxis_A, XDir);
  digitalWrite(Yaxis_A, YDir);

  if (Xpulses < Ypulses)
  {
    for (unsigned long x = 0; x < Xpulses; x++) //target 200 cycles/pulse, -13 per loop iteration  ~ 190kHz
    {
      PORTA |= _BV(PA3);//1 cycle
      PORTA |= _BV(PA2);//1 cycle
      NOM;//10 cycles
      NOM; NOP; NOP; //12cycles
      PORTA &= ~(_BV(PA3));//1 cycle
      PORTA &= ~(_BV(PA2));//1 cycle
      NOM; NOM; NOM; NOM; NOM; //50cyles
    }//2+10+12+2+50+13(for loop)=89
    for (unsigned long x = 0; x < (Ypulses - Xpulses); x++) //target 200 cycles/pulse, -13 per loop iteration  ~ 190kHz
    {
      PORTA |= _BV(PA2);//1 cycle
      NOM;//10 cycles
      NOM; NOP; NOP; //12cycles
      PORTA &= ~(_BV(PA2));//1 cycle
      NOM; NOM; NOM; NOM; NOM; //50cyles
    }///1+10+12+1+50+13(for loop)=87
  }
  else if (Xpulses > Ypulses)
  {
    for (unsigned long x = 0; x < Ypulses; x++) //target 200 cycles/pulse, -13 per loop iteration  ~ 190kHz
    {
      PORTA |= _BV(PA3);//1 cycle
      PORTA |= _BV(PA2);//1 cycle
      NOM;//10 cycles
      NOM; NOP; NOP; //12cycles
      PORTA &= ~(_BV(PA3));//1 cycle
      PORTA &= ~(_BV(PA2));//1 cycle
      NOM; NOM; NOM; NOM; NOM; //50cyles
    }//2+10+12+2+50+13(for loop)=89
    for (unsigned long x = 0; x < (Xpulses - Ypulses); x++) //target 200 cycles/pulse, -13 per loop iteration  ~ 190kHz
    {
      PORTA |= _BV(PA3);//1 cycle
      NOM;//10 cycles
      NOM; NOP; NOP; //12cycles
      PORTA &= ~(_BV(PA3));//1 cycle
      NOM; NOM; NOM; NOM; NOM; //50cyles
    }//1+10+12+1+50+13(for loop)=87
  }
  else
  {
    for (unsigned long x = 0; x < Xpulses; x++) //target 200 cycles/pulse, -13 per loop iteration  ~ 190kHz
    {
      PORTA |= _BV(PA3);//1 cycle
      PORTA |= _BV(PA2);//1 cycle
      NOM;//10 cycles
      NOM; NOP; NOP; //12cycles
      PORTA &= ~(_BV(PA3));//1 cycle
      PORTA &= ~(_BV(PA2));//1 cycle
      NOM; NOM; NOM; NOM; NOM; //50cyles
    }//2+10+12+2+50+13(for loop)=89
  }
  Y_WaitForMove();
  X_WaitForMove();
}

void X_WaitForMove()
{
  delay(300);
  while (digitalRead(Xaxis_HLFB) == HIGH)
  { }
}

void Y_WaitForMove()
{
  delay(300);
  while (digitalRead(Yaxis_HLFB) == HIGH)
  { }
}
void Z_WaitForMove()
{
  delay(300);
  while (digitalRead(Zaxis_HLFB) == HIGH)
  { }
}

