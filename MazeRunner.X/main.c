/*
 * File:   main.c
 * Author: olt13
 *
 * Created on 17 March 2020, 12:54
 */


#include "xc.h"
#include "Robot API/allcode_api.h"

#define LEDWAIT 2000 // Flash the LEDs
#define DRIVEWAIT 100 // Time to let the wheels adjust according to encorder readings
#define TIMEOUT 1500 // TIMEOUT for turning

typedef struct {
    //walls: 1 is wall 0 is no wall
    int left;
    int right;
    int top;
    int bottom;
    //if robot has seen it 1/0
    int visited;
    int nest;
} Square;

typedef struct {
    Square grid[5][5];
    int robotPos[2];
    int orientation; // 1:top, 2:right, 3:left, 4:down
} Grid;

Grid map;

void placeWalls(int x, int y) {
    int dist = 250;
    unsigned short front, left, right;
    front = FA_ReadIR(IR_FRONT);
    left = FA_ReadIR(IR_LEFT);
    right = FA_ReadIR(IR_RIGHT);

    switch(map.orientation) { // Place walls depending on current orientation
        case 1: // TOP
            if (front > dist) {
                map.grid[x][y].top = 1;
            }
            if (left > dist) {
                map.grid[x][y].left = 1;
            }
            if (right > dist) {
                map.grid[x][y].right = 1;
            }
            map.grid[x][y].bottom = 0;
            break;
        case 2: // RIGHT
            if (front > dist) {
                map.grid[x][y].right = 1;
            }
            if (left > dist) {
                map.grid[x][y].top = 1;
            }
            if (right > dist) {
                map.grid[x][y].bottom = 1;
            }
            map.grid[x][y].left = 0;
            break;
        case 3: // LEFT
            if (front > dist) {
                map.grid[x][y].left = 1;
            }
            if (left > dist) {
                map.grid[x][y].bottom = 1;
            }
            if (right > dist) {
                map.grid[x][y].top = 1;
            }
            map.grid[x][y].right = 0;
            break;
        case 4: // BOTTOM
            if (front > dist) {
                map.grid[x][y].bottom = 1;
            }
            if (left > dist) {
                map.grid[x][y].right = 1;
            }
            if (right > dist) {
                map.grid[x][y].left = 1;
            }
            map.grid[x][y].top = 0;
            break;
    }
}

int mapping() {
    switch(map.orientation) { //Move the virtual position in the direction of the orientation
        case 1: // TOP
            map.robotPos[1]++;
            break;
        case 2: // RIGHT
            map.robotPos[0]++;
            break;
        case 3: // LEFT
            map.robotPos[0]--;
            break;
        case 4: // BOTTOM
            map.robotPos[1]--;
            break;
    }
    
    if (map.grid[map.robotPos[0]][map.robotPos[1]].visited == 0) { // If square unvisited then place the walls
        map.grid[map.robotPos[0]][map.robotPos[1]].visited = 1;
        placeWalls(map.robotPos[0],map.robotPos[1]);
        FA_DelayMillis(200);
        return 1; // Return that it has visited a new square to add to new square counter
    }
    return 0;
}

void changeOrientation(int turn) {
    switch(map.orientation) { // Turning changes the orientation of the robot - new orientation also depends on current orientation
        case 1: // TOP
            if (turn == 1){
                map.orientation = 2; // RIGHT
            } else if(turn == 2) {
                map.orientation = 3; // LEFT
            } else if(turn == 3) {
                map.orientation = 4; // BOTTOM
            }
            break;
        case 2: // RIGHT
            if (turn == 1){
                map.orientation = 4; // BOTTOM
            } else if(turn == 2) {
                map.orientation = 1; // TOP
            } else if(turn == 3) {
                map.orientation = 3; // LEFT
            }
            break;
        case 3: // LEFT
            if (turn == 1){
                map.orientation = 1; // TOP
            } else if(turn == 2) {
                map.orientation = 4; // BOTTOM
            } else if(turn == 3) {
                map.orientation = 2; // RIGHT
            }
            break;
        case 4: // BOTTOM
            if (turn == 1){
                map.orientation = 3; // LEFT
            } else if(turn == 2) {
                map.orientation = 2; // RIGHT
            } else {
                map.orientation = 4; // TOP
            }
            break;
    }
}

int nestDetect() { // Function for detecting nest areas
    static int state = 1;
    unsigned short lightReading = FA_ReadLight();
    static unsigned short timer;
    int result = 0;
    
    switch(state) {
        case 1: // Check if enter a nest area
            if (lightReading < 1000) {
                state = 2;
                timer = FA_ClockMS() + LEDWAIT;
                result = 1;
                map.grid[map.robotPos[0]][map.robotPos[1]].nest = 1; // Tell the stored map there is a nest in this location
            }
            break;
        case 2: // Flash the LED
            FA_LEDOn(0);
            while (FA_ClockMS() < timer) {
                return result;
            }
            FA_LEDOff(0);
            state = 3;
            break;
        case 3: // Check if left the nest area
            if (lightReading > 1500) state = 1;
            break;
    }
    return result;
}

int lineDetect() {
    static int state = 1;
    unsigned short averageReading = (FA_ReadLine(0) + FA_ReadLine(1))/2;
    
    switch(state) {
        case 1: // Check if over a line
            if (averageReading < 200) {
                state = 2;
            }
            break;
        case 2: // return that line crossed
            state = 3;
            return 1;
        case 3: // Check that moved over the line
            if (averageReading > 400) {
                state = 1;
            }
            break;
    }
    return 0;
}

int turning(int angle, int dir) { // dir = 1/-1 right/left
    const float tickL = 1.8; // Constants for tick per degree rotation
    const float tickR = 1.7;
    int targetL = angle * tickL; // Calculate targets
    int targetR = angle * tickR;
    static unsigned short timeout;
    static int state = 1;
    
    switch(state) {
        case 1: // Start turning
            FA_SetMotors(15*dir,-15*dir); // dir is either -1 or 1 and sets the direction of the turn
            state = 2;
            timeout = FA_ClockMS() + TIMEOUT;
            break;
        case 2: // Check if target is reached
            if (FA_ReadEncoder(0) > targetL && FA_ReadEncoder(1) > targetR) state = 3;
            if (FA_ClockMS() > timeout) state = 3;
            break;
        case 3:
            FA_SetMotors(0,0);
            FA_ResetEncoders();
            FA_DelayMillis(500);
            state = 1;
    }
    return state;
}

void driveDistance(int dist) {
    int power = 20;
    static int slavePower, masterPower;
    static int state = 1;
    const int k = 5; // proportional constant
    unsigned short error;
    slavePower = power;
    masterPower = power;
    static unsigned short timer;
    const int distance = 100; // for the IR sensors
    const float distPerTick = 2.8; // in mm
    static unsigned short totalTicks = 0;
    int target = dist * distPerTick;
    
    switch(state) {
        case 1: // Start forward driving
            FA_SetMotors(masterPower, slavePower);
            totalTicks += FA_ReadEncoder(0);
            error = FA_ReadEncoder(0) - FA_ReadEncoder(1);
            slavePower += error/k;
            FA_ResetEncoders();
            timer = FA_ClockMS() + DRIVEWAIT;
            state = 2;
            break;
        case 2: // Wait for: timer to correct motors / going towards a wall / reached target
            if (FA_ClockMS() > timer) { // Go back to check if motors are running together
                state = 1;
            }
            if (FA_ReadIR(IR_FRONT) > 700) { // Heading straight to a wall
                state = 3;
            } else if (FA_ReadIR(IR_FRONT_RIGHT) > 500) { // Heading to a side wall to a wall
                FA_SetMotors(masterPower, slavePower + k);
            } else if (FA_ReadIR(IR_FRONT_LEFT) > 500) {
                FA_SetMotors(masterPower + k, slavePower);
            }
            if (totalTicks > target) { // Reached target distance
                state = 3;
            }
            break;
        case 3: // Stop and check the available routes - left first, then forward, then right, then turn around
            FA_SetMotors(0,0);
            FA_DelayMillis(200);
            totalTicks = 0;
            if (FA_ReadIR(IR_LEFT) < distance) state = 4; // GO LEFT
            else if (FA_ReadIR(IR_FRONT) < distance) state = 1; // GO FORWARD
            else if (FA_ReadIR(IR_RIGHT) < distance) state = 5; // GO RIGHT
            else state = 6; // TURN AROUND
            break;
        case 4: // Turn left
            if (turning(90,-1) == 1){
                state = 1;
                changeOrientation(2);
            }
        case 5: // Turn Right
            if (turning(90,1) == 1) {
                state = 1;
                changeOrientation(1);
            }
            break;
        case 6: // Turn 180
            if (turning(180,1) == 1) {
                state = 1;
                changeOrientation(3);
            }
    }
}

void printMaze() {
    int x,y;
    for (x = 0; x < 5; x++) {
        for (y = 0; y < 5; y++) {
            if(map.grid[x][y].top == 1) {
                if (y == 0) FA_LCDLine(x*6, y*6,(x*6)+4,y*6);
                else FA_LCDLine(x*6, (y*6)-1,(x*6)+4,(y*6)-1);
            }
            if(map.grid[x][y].bottom == 1) {
                if (y == 0) FA_LCDLine(x*6, (y*6)+6,(x*6)+4,(y*6)+6);
                else FA_LCDLine(x*6, (y*6)+5,(x*6)+4,(y*6)+5);
            }
            if(map.grid[x][y].left == 1) {
                if (x == 0) FA_LCDLine(x*6, y*6,x*6,(y*6)+4);
                else FA_LCDLine((x*6)-1, y*6,(x*6)-1,(y*6)+4);
            }
            if(map.grid[x][y].right == 1) {
                if (x == 0) FA_LCDLine((x*6)+6, y*6,(x*6)+6,(y*6)+4);
                else FA_LCDLine((x*6)+5, y*6,(x*6)+5,(y*6)+4);
            }
            if (map.grid[x][y].nest == 1) {
                FA_LCDPrint("x",1,x*6+1,y*6-2,0,1); // print an x if there is a nest area
            }
        }
    }
}

int main(void) {
    FA_RobotInit();
    FA_LCDBacklight(50);
    
    /* INITIALISE MAZE WITH OUTSIDE WALLS */
    int x,y;
    for (x = 0; x < 5; x++) {
        map.grid[x][0].top = 1;
        map.grid[x][4].bottom = 1;
    }
    for (y = 0; y < 5; y++) {
        map.grid[0][y].left = 1;
        map.grid[4][y].right = 1;
    }
    
    /* INITIALISE START POSITION AND PLACE INITIAL SQUARE WALLS */
    map.robotPos[0] = 4;
    map.robotPos[1] = 2;
    map.orientation = 1;
    map.grid[4][2].visited = 1;
    placeWalls(4,2);
    
    int squareCounter = 1;
    int nestCounter = 0;
    
    /* MAIN LOOP */
    while(1) {
        driveDistance(155); // Drive 15cm unless blocked
        if (lineDetect() == 1) squareCounter += mapping(); // If crossed a line, update the map and the square counter
        nestCounter += nestDetect(); // If detected a nest
        printMaze();

        if (squareCounter == 25) break; // If explored whole maze
    }
    
    FA_SetMotors(0,0);
    printMaze();
    FA_LCDNumber(nestCounter,40,15,2,0); // Display the number of the nest areas in the maze to the right of the maze display
    
    while(1) { // Display information until reset
        ;
    }
    
    return 0;
}
