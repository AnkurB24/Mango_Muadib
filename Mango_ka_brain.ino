#include "Arduino.h"
#include "DCMDriverL298.h"
#include "NewPing.h"

// Pin Definitions
#define L298_PIN_INT1 2
#define L298_PIN_ENB 6
#define L298_PIN_INT2 3
#define L298_PIN_ENA 5
#define L298_PIN_INT3 4
#define L298_PIN_INT4 7
#define ultra_PIN_TRIG 9
#define ultra_PIN_ECHO 8
#define IR_1_PIN_OUT 10
#define IR_2_PIN_OUT 11

// Global variables and defines
#define MAZE_WIDTH 20
#define MAZE_HEIGHT 20
#define UNEXPLORED 255 // Changed to use a larger value for unexplored cells
#define WALL 1
#define OPEN_SPACE 0 // Changed to use 0 for open space
#define START_POINT 1 // Changed to use 1 for start point

// Define a structure to represent a cell
struct Cell {
    byte x;
    byte y;
    short int distance;
    byte walls; // Represent walls with bits: bit 0 = north, bit 1 = east, bit 2 = south, bit 3 = west
};

// Wall constants
#define NORTH 1
#define EAST 2
#define SOUTH 4
#define WEST 8

// object initialization
DCMDriverL298 dcMotorDriverL298(L298_PIN_ENA, L298_PIN_INT1, L298_PIN_INT2, L298_PIN_ENB, L298_PIN_INT3, L298_PIN_INT4);
NewPing hcsr04(ultra_PIN_TRIG, ultra_PIN_ECHO);

// Define maze representation
Cell maze[MAZE_HEIGHT][MAZE_WIDTH];
int start_x = 0; // Adjust according to your maze layout
int start_y = 0;
int current_x = 0; // Variables to keep track of current position
int current_y = 0;

void setup() {
    //Serial.begin(9600);
    pinMode(IR_1_PIN_OUT, INPUT);
    pinMode(IR_2_PIN_OUT, INPUT);
    initializeMaze();
    floodfill();
    //printMaze(); // For debugging
    followPath();
}

void loop() {
    // Your main loop code goes here
}

void initializeMaze() {
    for (int i = 0; i < MAZE_HEIGHT; i++) {
        for (int j = 0; j < MAZE_WIDTH; j++) {
            maze[i][j].x = j;
            maze[i][j].y = i;
            maze[i][j].distance = UNEXPLORED;
            maze[i][j].walls = 0; // No walls initially
        }
    }
    maze[start_y][start_x].distance = START_POINT;
}

void floodfill() {
    // Define pointers for current and next levels
    Cell* currentLevel = new Cell[MAZE_WIDTH * MAZE_HEIGHT];
    Cell* nextLevel = new Cell[MAZE_WIDTH * MAZE_HEIGHT];
    int currentLevelSize = 1; // Initialize with 1 for start point
    int nextLevelSize = 0;

    // Initialize start point in the current level
    currentLevel[0] = maze[start_y][start_x];

    // Initialize distance for start point
    int distance = 0;

    while (currentLevelSize > 0) {
        for (int i = 0; i < currentLevelSize; i++) {
            Cell cell = currentLevel[i];
            if (cell.distance == UNEXPLORED) {
                cell.distance = distance;
                maze[cell.y][cell.x].distance = distance;
                // Add unexplored neighbors to the next level
                if (!(cell.walls & NORTH) && cell.y > 0) {
                    nextLevel[nextLevelSize++] = maze[cell.y - 1][cell.x];
                }
                if (!(cell.walls & EAST) && cell.x < MAZE_WIDTH - 1) {
                    nextLevel[nextLevelSize++] = maze[cell.y][cell.x + 1];
                }
                if (!(cell.walls & SOUTH) && cell.y < MAZE_HEIGHT - 1) {
                    nextLevel[nextLevelSize++] = maze[cell.y + 1][cell.x];
                }
                if (!(cell.walls & WEST) && cell.x > 0) {
                    nextLevel[nextLevelSize++] = maze[cell.y][cell.x - 1];
                }
            }
        }
        // Swap current and next level pointers
        Cell* temp = currentLevel;
        currentLevel = nextLevel;
        nextLevel = temp;
        currentLevelSize = nextLevelSize;
        nextLevelSize = 0;
        distance++;
    }

    // Free allocated memory
    delete[] currentLevel;
    delete[] nextLevel;
}

/*void printMaze() {
    for (int i = 0; i < MAZE_HEIGHT; i++) {
        for (int j = 0; j < MAZE_WIDTH; j++) {
            Serial.print("Cell (");
            Serial.print(maze[i][j].x);
            Serial.print(",");
            Serial.print(maze[i][j].y);
            Serial.print("), Distance: ");
            Serial.print(maze[i][j].distance);
            Serial.print(", Walls: ");
            Serial.print(maze[i][j].walls, BIN);
            Serial.println();
        }
    }
}*/

void followPath() {
    int x = start_x;
    int y = start_y;

    while (maze[y][x].distance != 0) { // While not at the destination
        // Read sensor values
        int frontDistance = hcsr04.ping_cm();
        int leftSensor = digitalRead(IR_1_PIN_OUT);
        int rightSensor = digitalRead(IR_2_PIN_OUT);

        // Update maze based on sensor readings
        if (frontDistance < 5) {
            // Update maze with wall in front
            maze[y][x].walls |= EAST;
            maze[y][x + 1].walls |= WEST;
        }
        if (leftSensor == LOW) {
            // Update maze with wall on left
            maze[y][x].walls |= SOUTH;
            maze[y + 1][x].walls |= NORTH;
        }
        if (rightSensor == LOW) {
            // Update maze with wall on right
            maze[y][x].walls |= NORTH;
            maze[y - 1][x].walls |= SOUTH;
        }

        // Perform pathfinding
        // Move to the adjacent cell with the lowest floodfill value
        // Prioritize moving towards cells with lower values to reach the destination
        int minNeighborValue = 255; // Initialize with a high value
        int minNeighborX = x;
        int minNeighborY = y;

        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                int neighborX = x + i;
                int neighborY = y + j;
                if (neighborX >= 0 && neighborX < MAZE_WIDTH && neighborY >= 0 && neighborY < MAZE_HEIGHT &&
                    maze[neighborY][neighborX].distance < minNeighborValue) {
                    minNeighborValue = maze[neighborY][neighborX].distance;
                    minNeighborX = neighborX;
                    minNeighborY = neighborY;
                }
            }
        }

        // Move to the cell with the lowest floodfill value
        x = minNeighborX;
        y = minNeighborY;

        // Output movement (replace with motor control functions)
        //Serial.print("Move to: ");
        //Serial.print(x);
        //Serial.print(", ");
        //Serial.println(y);
        delay(1000); // Adjust delay as needed
    }
}
void m_fwd(){
  dcMotorDriverL298.setMotorA(255,1);
  dcMotorDriverL298.setMotorB(255,0);
}

void m_bwd(){
  dcMotorDriverL298.setMotorA(255,0);
  dcMotorDriverL298.setMotorB(255,1);
}

void m_stp(){
  dcMotorDriverL298.stopMotorA();
  dcMotorDriverL298.stopMotorB();
}

void m_lft(){
  dcMotorDriverL298.setMotorA(255,0);
  dcMotorDriverL298.setMotorB(255,0);
  delay(500);
  dcMotorDriverL298.stopMotorA();
  dcMotorDriverL298.stopMotorB();
}

void m_rgt(){
  dcMotorDriverL298.setMotorA(255,1);
  dcMotorDriverL298.setMotorB(255,1);
  delay(500);
  dcMotorDriverL298.stopMotorA();
  dcMotorDriverL298.stopMotorB();
}


//Sensor Functions

int ult_dtc(){
  delay(100);
  int distance = ultrasonic.read();
  //Serial.println(distance);
  if (distance >= 4){
    return 1;
  }
  else {
    return 0;
  }

}

int IR1_dtc(){
  int ir1stat = !digitalRead(IR_1_PIN_OUT);

  return ir1stat;
}

int IR2_dtc(){
  int ir2stat = !digitalRead(IR_2_PIN_OUT);

  return ir2stat;
}
