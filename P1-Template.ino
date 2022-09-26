#include <ax12.h>
#include "poses.h"
#include "robot.h"
#include <avr/interrupt.h> 
#include <avr/io.h>


// This function will only run once, after each powerup or reset of the Arduino board.
void setup()
{
  /* Init Robot */
  ROBOT_Init();

  /* Open serial port */
  //Serial:used for communication between the Arduino board and a computer or other devices.
  Serial.begin(115200);
  delay (100);   
  Serial.println("###########################");    
  Serial.println("Serial Communication Established.");    
  Serial.println("###########################"); 
  delay (2000);    //for stability on programming
  
  /* Start menu options */
  MenuOptions();
}

/*******************************************
 *******************************************/

void loop()
{
  unsigned int  tmp[SERVOCOUNT-1];
  double        tmpJointAngles[DOFs];
  double        tmpCoords[3];
  
  for (int i = 0 ; i < SERVOCOUNT - 1; i++)
    tmp[i]=0;
    
  if (Serial.available() > 0 )
  {
    int inByte = Serial.read();

    switch (inByte) {
      case '0':
        SERVOS_ServosOff();
        break;
      case '1':
        SERVOS_ServosOn();
        break;
      case '2':
        ROBOT_GetJointsPos(tmpJointAngles);
        Serial.print("Base: ");
        Serial.println(tmpJointAngles[0]);
        Serial.print("Shoulder: ");
        Serial.println(tmpJointAngles[1]);
        Serial.print("Elbow: ");
        Serial.println(tmpJointAngles[2]);
        Serial.print("Wrist: ");
        Serial.println(tmpJointAngles[3]);
        Serial.print("WristRot: ");
        Serial.println(tmpJointAngles[4]);
        break;
      case '3':  
        ROBOT_GripperClose();
        break;     
      case '4':
        ROBOT_GripperOpen();
        break;
      case '5':   
        TestAllJoints();
        break;     
      case '6':
        MoveSpecificJoint();
        break;
      case 'A':
        P1Solution();
        break;
      case 'B':
        Check();
        break;
      case 'C':
        PruebaSolucion();
      break;
    }

    /* Run menu options again */
    MenuOptions();
  }
}

/*******************************************
 *******************************************/

void MenuOptions()
{ 
    Serial.println("###########################"); 
    Serial.println("Please enter option 1-5 to run individual tests again.");     
    Serial.println("0) Relax Servos");        
    Serial.println("1) Hold Servos");        
    Serial.println("2) Get Joints Pos");  
    Serial.println("3) Gripper Close");  
    Serial.println("4) Gripper Open");  
    Serial.println("5) TestAllJoints");  
    Serial.println("6) MoveSpecificJoint"); 
    Serial.println("A) P1 Solution");  
    Serial.println("B) Check");  
    Serial.println("C) Trayectoria 2");  
    Serial.println("###########################"); 
}


/*******************************************
 *******************************************/

void TestAllJoints ( void )
{
  double fIncQ      = 0.01;
  uint16_t unSteps  = 100;
  uint8_t i, j;

  /* Get a correct Position for testing all joints */
  ROBOT_SetSingleTrajectory( m_fCoordTest, 1000, LINEAR );
  delay(1500);

  /* Check ALL JONTS but GRIPPER*/
  for ( i = 0 ; i < 5 ; i++ )
  {
    for ( j = 0 ; j < unSteps ; j++ )
    {
      ROBOT_SetJointPos(i, m_fCoordTest[i] + (double) j * fIncQ );
      delay(20);
    }

    for ( j = unSteps ; j > 0 ; j-- )
    {
      ROBOT_SetJointPos(i, m_fCoordTest[i] + (double) j * fIncQ );
      delay(20);
    }
  }
  
  /* Test GRIPPER*/
  ROBOT_GripperClose();
  delay(500);
  ROBOT_GripperOpen();
  delay(500);
  
  /* Go for a RELAX Position */
  ROBOT_SetSingleTrajectory( m_fCoordRelax, 1000, LINEAR );
}

/*******************************************
 *******************************************/

int8_t MoveSpecificJoint ( void )
{

  Serial.println("Base: 0");
  Serial.println("Shoulder: 1");
  Serial.println("Elbow: 2");
  Serial.println("Wrist: 3");
  Serial.println("Wrist Rot: 4");
  Serial.println("");
  Serial.println("Joint to move?: ");
  
  while (Serial.available() <= 0 );
  uint8_t unJoint = Serial.parseInt();

  Serial.print("Joint selected: ");
  Serial.println(unJoint);
  Serial.println("");

  switch(unJoint)
  {
    case BASE:
      Serial.println("Base:  [-2.62, 2.62]");
      break;
    case SHOULDER:
      Serial.println("Shoulder: [-0.33, 2.97]");
      break;
    case ELBOW:
      Serial.println("Elbow: [-2.89, 0.26]");
      break;
    case WRIST:
      Serial.println("Wrist: [-1.83, 1.86]");
      break;
    case WRIST_ROT:
      Serial.println("Wrist Rot: [-2.62, 2.62]");
      break;
    default:
      return -1;
      break;
  }

  Serial.print("Position to move?: ");
  
  while (Serial.available() <= 0 );
  float fPosition = Serial.parseFloat();
  
  Serial.println("");

  Serial.print("Moving Joint: ");
  Serial.print(unJoint);
  Serial.print(" to position: ");
  Serial.println(fPosition);
      
  ROBOT_SetJointPos(unJoint, fPosition );
}

/*******************************************
 *******************************************/

void P1Solution ( void )
{
  double        tmpJointAngles[DOFs];
  /* START CODE TO BE IMPLEMENTED BY THE STUDENTS */
  double fIncQ      = 0.01;
  uint16_t unSteps  = 100;
  uint8_t i, j;
  ROBOT_GripperOpen();
  delay(500);
  ROBOT_SetSingleTrajectory( m_fCoordRelax, 1000, CUBIC1 );
  delay(2500);
  ROBOT_SetSingleTrajectory( m_fCoordRelax1, 2000, CUBIC1 );
  delay(2500);
  ROBOT_GripperClose();
  delay(1500);
  ROBOT_SetDoubleTrajectory (m_fCoordRelax3, m_fCoordRelax2, 2000, 3000, CUBIC2);
  delay(5000);
  ROBOT_GripperOpen();
  delay(1000);
  ROBOT_SetSingleTrajectory( m_fCoordRelax, 2000, CUBIC1 );

  ROBOT_GetJointsPos(tmpJointAngles);
  Serial.print("Base: ");
        Serial.println(tmpJointAngles[0]);
        Serial.print("Shoulder: ");
        Serial.println(tmpJointAngles[1]);
        Serial.print("Elbow: ");
        Serial.println(tmpJointAngles[2]);
        Serial.print("Wrist: ");
        Serial.println(tmpJointAngles[3]);
        Serial.print("WristRot: ");
        Serial.println(tmpJointAngles[4]);

  /* Get a correct Position for testing all joints */
  


  /* ROBOT_SetSingleTrajectory( m_fCoordTest, 1000, LINEAR );
  delay(1500);
  Serial.print("POS 1"); */
  /* dxlTorqueOnAll(); */


  /* Check ALL JONTS but GRIPPER
  for ( i = 0 ; i < 5 ; i++ )
  {
    for ( j = 0 ; j < unSteps ; j++ )
    {
      ROBOT_SetJointPos(i, m_fCoordTest[i] + (double) j * fIncQ );
      delay(20);
    }

    for ( j = unSteps ; j > 0 ; j-- )
    {
      ROBOT_SetJointPos(i, m_fCoordTest[i] + (double) j * fIncQ );
      delay(20);
    }
  }
  

  ROBOT_GripperClose();
  delay(500);
  ROBOT_GripperOpen();
  delay(500);
  
 
  ROBOT_SetSingleTrajectory( m_fCoordRelax, 1000, LINEAR );*/
}
void Check ( void )
{
  double        tmpJointAngles[DOFs];
  /* START CODE TO BE IMPLEMENTED BY THE STUDENTS */
  double fIncQ      = 0.01;
  uint16_t unSteps  = 100;
  delay(100);
  ROBOT_GetJointsPos(tmpJointAngles);
  Serial.print("Base: ");
        Serial.println(tmpJointAngles[0]);
        Serial.print("Shoulder: ");
        Serial.println(tmpJointAngles[1]);
        Serial.print("Elbow: ");
        Serial.println(tmpJointAngles[2]);
        Serial.print("Wrist: ");
        Serial.println(tmpJointAngles[3]);
        Serial.print("WristRot: ");
        Serial.println(tmpJointAngles[4]);
}
void PruebaSolucion ( void )
{
  double        tmpJointAngles[DOFs];
  /* START CODE TO BE IMPLEMENTED BY THE STUDENTS */
  double fIncQ      = 0.01;
  uint16_t unSteps  = 100;
  delay(100);
  ROBOT_GetJointsPos(tmpJointAngles);
  Serial.print("Base: ");
        Serial.println(tmpJointAngles[0]);
        Serial.print("Shoulder: ");
        Serial.println(tmpJointAngles[1]);
        Serial.print("Elbow: ");
        Serial.println(tmpJointAngles[2]);
        Serial.print("Wrist: ");
        Serial.println(tmpJointAngles[3]);
        Serial.print("WristRot: ");
        Serial.println(tmpJointAngles[4]);
}
  /* END CODE TO BE IMPLEMENTED BY THE STUDENTS */
  



/*******************************************
 *******************************************/
