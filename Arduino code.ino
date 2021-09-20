#include <SoftwareSerial.h>
#include <PololuQik.h>
#include <QTRSensors.h>

#define numberOfSensors 6
#define samplesPerSensor 4

//delay times
#define checkTime 30
#define timeBeforeTurn 200

//thresholds for determining colors
#define blackThreshold 400
#define whiteThreshold 100

//the motor speeds
#define M0Speed 27 //left motor
#define M1Speed -27 //right motor

//max size of the path
#define pathSize 100

//sets up the sensor array
QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5}, numberOfSensors, samplesPerSensor, QTR_NO_EMITTER_PIN);

//sets up the motor controller
PololuQik2s9v1 qik(2, 3, 4);

//the array where the sensor values are stored
unsigned int sensorValues[numberOfSensors];

boolean mazeSolved;

//stores all the turns that the robot takes when it is solving the maze
char path[pathSize] = {};

//stores the path after it has been optimized
char optimizedPath[pathSize] = {};

int pathLength;
int optimizedPathLength;

boolean turnDetected() {
  qtra.read(sensorValues);
  return sensorValues[0] > blackThreshold || sensorValues[5] > blackThreshold;
}

boolean isLeftTurn() {
  qtra.read(sensorValues);
  return sensorValues[0] > blackThreshold;
}

boolean isRightTurn() {
  qtra.read(sensorValues);
  return sensorValues[5] > blackThreshold;
}

boolean veeringLeft() {
  qtra.read(sensorValues);
  return sensorValues[2] < blackThreshold && sensorValues[3] > blackThreshold;
}

boolean veeringRight() {
  qtra.read(sensorValues);
  return sensorValues[3] < blackThreshold && sensorValues[2] > blackThreshold;
}

boolean allBlack() {
  qtra.read(sensorValues);
  return sensorValues[0] > blackThreshold && sensorValues[1] > blackThreshold &&
         sensorValues[2] > blackThreshold && sensorValues[3] > blackThreshold &&
         sensorValues[4] > blackThreshold && sensorValues[5] > blackThreshold;
}

boolean allWhite() {
  qtra.read(sensorValues);
  return sensorValues[0] < whiteThreshold && sensorValues[1] < whiteThreshold &&
         sensorValues[2] < whiteThreshold && sensorValues[3] < whiteThreshold &&
         sensorValues[4] < whiteThreshold && sensorValues[5] < whiteThreshold;
}

boolean centeredOnLine() {
  qtra.read(sensorValues);
  return sensorValues[0] < whiteThreshold && sensorValues[1] < whiteThreshold &&
         sensorValues[2] > blackThreshold && sensorValues[3] > blackThreshold &&
         sensorValues[4] < whiteThreshold && sensorValues[5] < whiteThreshold;
}

void turnLeft() {
  qik.setM0Speed(-M0Speed);
  qik.setM1Speed(M1Speed);
  //if the turn is started on a black line, it needs to get off the line first
  while (centeredOnLine());
  delay(200);
  while (!centeredOnLine());
  qik.setM0Speed(M0Speed);
  path[pathLength] = 'L';
  pathLength++;
}

void turnRight() {
  qik.setM0Speed(M0Speed);
  qik.setM1Speed(-M1Speed);
  //if the turn is started on a black line, it needs to get off the line first
  while (centeredOnLine());
  delay(200);
  while (!centeredOnLine());
  qik.setM1Speed(M1Speed);
  path[pathLength] = 'R';
  pathLength++;
}

void turnBack() {
  qik.setM0Speed(-M0Speed);
  qik.setM1Speed(M1Speed);
  //if the turn is started on a black line, it needs to get off the line first
  while (centeredOnLine());
  delay(200);
  while (!centeredOnLine());
  qik.setM0Speed(M0Speed);
  path[pathLength] = 'B';
  pathLength++;
}

void turnBackForOptimized() {
  qik.setM0Speed(-M0Speed);
  qik.setM1Speed(M1Speed);
  while (centeredOnLine());
  delay(200);
  while (!centeredOnLine());
  delay(200);
  while (centeredOnLine());
  delay(200);
  while (!centeredOnLine());
  qik.setM0Speed(M0Speed);
}

void goStraight() {
  qik.setM0Speed(M0Speed);
  qik.setM1Speed(M1Speed);
  path[pathLength] = 'S';
  pathLength++;
}

void mazeFinished() {
  mazeSolved = true;
  qik.setM0Speed(0);
  qik.setM1Speed(0);
  delay(2000);
  qik.setM0Speed(50);
  qik.setM1Speed(50);
  delay(3000);
  qik.setM0Speed(0);
  qik.setM1Speed(0);
}

void followLine() {
  while (veeringLeft()) {
    qik.setM1Speed(0);
  }
  qik.setM1Speed(M1Speed);
  
  while (veeringRight()) {
    qik.setM0Speed(0);
  }
  qik.setM0Speed(M0Speed);
}

void checkForTurns() {
  if (turnDetected()) {
    delay(checkTime);
    if (isLeftTurn()) {
      delay(timeBeforeTurn);
      if (allBlack()) {
        mazeFinished();
      } else {
        turnLeft();
      }
    } else if (isRightTurn()){
      delay(timeBeforeTurn);
      if (allBlack()) {
        mazeFinished();
      } else if (allWhite()) {
        turnRight();
      } else {
        goStraight();
      }
    }
  }
  
  if (allWhite()) {
    turnBack();
  }
}

void solveMaze() {
  qik.setM0Speed(M0Speed);
  qik.setM1Speed(M1Speed);
  while(mazeSolved == false) {
    followLine();
    checkForTurns();
  }
}

void nullify(char a[]) {
  for (int i=0; i<pathSize; i++) {
    a[i] = NULL;
  }
}

void optimizeMaze() {
  boolean changed = true;
  char a1[pathSize] = {};
  char a2[pathSize] = {};
  int tempLength = pathLength;
  optimizedPathLength = pathLength;
  int i = 0;
  int j = 0;
  
  //copies the path array
  for (i=0; i<pathLength; i++) {
    a1[i] = path[i];
  }
  
  while (changed == true) {
    changed = false;
    for (i=0, j=0; i<tempLength; i++, j++) {
      if (a1[i+1] != 'B') {
        a2[j] = a1[i];
      } else if (a1[i+1] == 'B') {
        
        if (a1[i] == 'L' && a1[i+2] == 'R') {
          a2[j] = 'B';
        } else if (a1[i] == 'L' && a1[i+2] == 'S') {
          a2[j] = 'R';
        } else if (a1[i] == 'R' && a1[i+2] == 'L') {
          a2[j] = 'B';
        } else if (a1[i] == 'S' && a1[i+2] == 'L') {
          a2[j] = 'R';
        } else if (a1[i] == 'S' && a1[i+2] == 'S') {
          a2[j] = 'B';
        } else if (a1[i] == 'L' && a1[i+2] == 'L') {
          a2[j] = 'S';
        }
        
        i+=2;
        optimizedPathLength -= 2;
      } //end if
      
    } //end for
    nullify(a1);
    tempLength = optimizedPathLength;
    
    for (i=0, j=0; i<tempLength; i++, j++) {
      if (a2[i+1] != 'B') {
        a1[j] = a2[i];
      } else if (a2[i+1] == 'B') {
        
        if (a2[i] == 'L' && a2[i+2] == 'R') {
          a1[j] = 'B';
        } else if (a2[i] == 'L' && a2[i+2] == 'S') {
          a1[j] = 'R';
        } else if (a2[i] == 'R' && a2[i+2] == 'L') {
          a1[j] = 'B';
        } else if (a2[i] == 'S' && a2[i+2] == 'L') {
          a1[j] = 'R';
        } else if (a2[i] == 'S' && a2[i+2] == 'S') {
          a1[j] = 'B';
        } else if (a2[i] == 'L' && a2[i+2] == 'L') {
          a1[j] = 'S';
        }
        
        i+=2;
        optimizedPathLength -= 2;
        changed = true;
      } //end if
      
    } //end for
    nullify(a2);
    tempLength = optimizedPathLength;
  } //end while
  
  for (i=0; i<optimizedPathLength; i++) {
    optimizedPath[i] = a1[i];
  }
}

void runOptimizedMaze() {
  int i = 0;
  qik.setM0Speed(M0Speed);
  qik.setM1Speed(M1Speed);
  
  while (i < optimizedPathLength) {
    followLine();
    if (turnDetected() || allWhite()) {
      delay(checkTime);
      if (optimizedPath[i] == 'L') {
        delay(timeBeforeTurn);
        turnLeft();
      } else if (optimizedPath[i] == 'R') {
        delay(timeBeforeTurn);
        turnRight();
      } else if (optimizedPath[i] == 'S') {
        delay(timeBeforeTurn);
        goStraight();
      } else if (optimizedPath[i] == 'B') {
        if (isLeftTurn()) {
          delay(timeBeforeTurn);
          turnBackForOptimized();
        } else {
          delay(timeBeforeTurn);
          turnBack();
        }
      }
      i++;
    } //end if
  } //end while
  
  while (!allBlack()) {
    followLine();
  }
  delay(timeBeforeTurn);
  mazeFinished();
}

void reset() {
  mazeSolved = false;
  nullify(path);
  nullify(optimizedPath);
  pathLength = 0;
  optimizedPathLength = 0;
}

void setup() {
  qik.init();
}

void loop() {
  reset();
  //if button pressed (delay 1 sec)
  solveMaze();
  delay(4000);
  //if button pressed again (delay 1 sec)
  optimizeMaze();
  runOptimizedMaze();
  delay(4000);
}
