/* 
 * File:   main.c
 * Author: mwauras
 *
 * Created on March 25, 2019, 10:08 PM
 */

#pragma config FNOSC = FRCPLL, POSCMOD = OFF
#pragma config FPLLIDIV = DIV_2, FPLLMUL = MUL_20
#pragma config FPBDIV = DIV_1, FPLLODIV = DIV_2
#pragma config FWDTEN = OFF, JTAGEN = OFF, FSOSCEN = OFF
//#pragma config WDTE = OFF 

#include <p32xxxx.h>
#include <stdio.h>
#include <stdlib.h>
#include <plib.h>
#include "analog_in.h"
#include "uart.h"

//INPUTS
//#define LEFT_MOTOR_OUTA     PORTBbits.RB3
//#define LEFT_MOTOR_OUTB     PORTCbits.RC0
//#define RIGHT_MOTOR_OUTA    PORTBbits.RB7
//#define RIGHT_MOTOR_OUTB    PORTBbits.RB8
#define IR_LEFT             (11)//AN11 on pin11            //LATAbits.LATA10
#define IR_FRONT             (10)//AN10 on pin14           //LATBbits.LATB9
#define IR_RIGHT             (9)//AN9 on pin15            //LATCbits.LATC7

//OUTPUTS
#define IR_EN1              LATAbits.LATA7
#define SW1                 PORTAbits.RA0
#define SW2                 PORTAbits.RA1
#define LED1                LATAbits.LATA2
#define LED2                LATAbits.LATA3

#define RIGHT_MOTORS_IN1    LATCbits.LATC2
#define RIGHT_MOTORS_IN2    LATCbits.LATC9
#define ML_IN1              LATBbits.LATB9
#define RIGHT_MOTOR_EN      LATBbits.LATB5
#define LEFT_MOTOR_EN       LATCbits.LATC3
#define IR_EN2              PORTCbits.RC6
#define IR_EN3              PORTCbits.RC9
#define LEFT_MOTORS_IN2     LATCbits.LATC4
#define LEFT_MOTORS_IN1     LATCbits.LATC5

#define IR_MAX              (420)
#define IR_MIN              (220)
#define IR_THRESH           (120)
#define IR_FRONT_THRESH     (180)
#define MAX_SPEED           (100)
#define RIGHT_MAX           MAX_SPEED//*0.73
#define LEFT_MAX            MAX_SPEED

//#define TURN_ROTATIONS      95
#define TURN_ROTATIONS      115
//#define FORWARD_ROTATIONS   265
#define FORWARD_ROTATIONS   305//310

#define SAMPLE_SIZE         20

#define D_PLUS  8
#define D_MINUS 9

#define EAST 0x08                   // east direction (binary representation)
#define WEST 0x04                   // west direction (binary representation)
#define NORTH 0x02                  // north direction (binary representation)
#define SOUTH 0x01                  // south direction (binary representation)

#define STEP_DELAY 250

volatile int32_t rightSpeed = RIGHT_MAX;
volatile int32_t leftSpeed = LEFT_MAX;

volatile uint8_t led1 = 0;
volatile uint32_t pulse_delay1 = 0;
volatile uint32_t timer3_val = 0;
volatile uint32_t leftPeriod = 0;

volatile uint8_t led2 = 0;
volatile uint32_t pulse_delay2 = 0;
volatile uint32_t timer4_val = 0;
volatile uint32_t rightPeriod = 0;

volatile uint32_t rightRotations = 0;
volatile uint32_t leftRotations = 0;

char str1[1000];
char str2[1000];
char str3[1000];

// this structure represents a physical cell on the maze
typedef struct{
    uint8_t index;                  // cell index : cell position on the maze
    uint8_t orientation;            // wall mapping of the cell (only 4 useful bits)
    int16_t pathValue;              // qualitative value of the path relative to the target cell
    uint8_t pending;                // shows which walls/directions have been explored from current cell
    uint8_t linkedCells[4];
} cell_type;

static cell_type cellContainer[256] = {0};   // global container for cells : analogous to the maze

static uint8_t pendingCells[256] = {0};
static uint8_t pendingCntr = 0;
static uint8_t lastKnownPendingIndex = 0;

//////////////////////////////////////////////////////////////////
// This keeps track of every cell the mouse has seen (without   // 
// retrace) and how it got there (N/E/S/W)                      //
//////////////////////////////////////////////////////////////////
static uint8_t pathTaken[256][2] = {{0,0}}; //keeps track of the left and rights taken to get to the current cell
static uint8_t stepNum=0; //the current index in the pathTaken array
//////////////////////////////////////////////////////////////////

static uint8_t rows = 0;            // number of rows
static uint8_t cols = 0;            // number of columns
static int numberOfCells = 0;
static uint8_t currFace = EAST;     // direction robot is facing : defaults to EAST
static uint8_t currCell = 0;        // index of current cell

static uint8_t startedMapping = 0;  // indicates whether we have started mapping

volatile uint32_t pwm_tmr;

/**
 * @brief rightDir gets the rightward direction from current cell
 * @return respective direction
 */
uint8_t rightDir(){
    switch (currFace){
        case EAST: return SOUTH;
        case WEST: return NORTH;
        case NORTH: return EAST;
        case SOUTH: return WEST;
        default: printf("Some error while getting right dir"); return currFace;
    }
}

/**
 * @brief leftDir gets leftward direction from current cell
 * @return respective direction
 */
uint8_t leftDir(){
    switch (currFace){
        case EAST: return NORTH;
        case WEST: return SOUTH;
        case NORTH: return WEST;
        case SOUTH: return EAST;
        default: printf("Some error while getting right dir"); return currFace;
    }
}

/**
 * @brief backDir gets backward direction from current cell
 * @return respective direction
 */
uint8_t backDir(){
    switch (currFace){
        case EAST: return WEST;
        case WEST: return EAST;
        case NORTH: return SOUTH;
        case SOUTH: return NORTH;
        default: printf("Some error while getting right dir"); return currFace;
    }
}

/**
 * @brief retrace synchronizes cell path values from the current cell
 * @param cellIndex     : cell index of current cell
 * @param prevPathVal   : path value of previous cell
 */
void retrace(uint8_t currCellIndex, int16_t currPathVal){
    sprintf(str1,"Remapping from cell %d, path value %d\n\r",currCellIndex, currPathVal);
    uart_write_string(str1);
    int i = 0;
    for (i = 0; i < 4; i++){
        if (cellContainer[currCellIndex].linkedCells[i] == 0) continue;
        else if (abs(currPathVal - cellContainer[cellContainer[currCellIndex].linkedCells[i]].pathValue) <= 1) continue;
        cellContainer[cellContainer[currCellIndex].linkedCells[i]].pathValue = currPathVal + 1;
        retrace(cellContainer[currCellIndex].linkedCells[i], cellContainer[cellContainer[currCellIndex].linkedCells[i]].pathValue);
    }
    return;
}

/**
 * @brief checkPathMapping updated and checks whether cell path
 *      values are in sync. If not, cell path values are recalibrated.
 * @param cell          : current cell the robot is on
 * @param currPathVal   : path value of previous cell
 */
void checkPathMapping(cell_type cell, int16_t prevPathVal){
    if (cell.pathValue == -1){
        cellContainer[cell.index].pathValue = prevPathVal + 1;
    } else if (cell.pathValue > prevPathVal + 1){
        retrace(cell.index, cell.pathValue);
    } else if (cell.pathValue < prevPathVal - 1){
        retrace(cell.index, cell.pathValue);
    }
}

void __ISR( _TIMER_1_VECTOR, ipl1auto) T1InterruptHandler (void){
    pwm_tmr++;
    if (rightSpeed >= 0){
        RIGHT_MOTORS_IN2 = 0;
        if (pwm_tmr < rightSpeed){
            RIGHT_MOTORS_IN1 = 1;
        } else {
            RIGHT_MOTORS_IN1 = 0;
        }
    } else {
        RIGHT_MOTORS_IN1 = 0;
        if (pwm_tmr < (-1)*rightSpeed){
            RIGHT_MOTORS_IN2 = 1;
        } else {
            RIGHT_MOTORS_IN2 = 0;
        }
    }
    
    if (leftSpeed >= 0){
        LEFT_MOTORS_IN2 = 0;
        if (pwm_tmr < leftSpeed){
            LEFT_MOTORS_IN1 = 1;
        } else {
            LEFT_MOTORS_IN1 = 0;
        }
    } else {
        LEFT_MOTORS_IN1 = 0;
        if (pwm_tmr < (-1)*leftSpeed){
            LEFT_MOTORS_IN2 = 1;
        } else {
            LEFT_MOTORS_IN2 = 0;
        }
    }
    
    if (pwm_tmr > MAX_SPEED*2) pwm_tmr = 0;
    // Clear the interrupt flag
    mT1ClearIntFlag();
}


void __ISR( _INPUT_CAPTURE_3_VECTOR, ipl1auto) IC3InterruptHandler (void){
    leftPeriod = IC3BUF;
    leftRotations = leftRotations + 1;
    TMR2 = 0;
    mIC3ClearIntFlag();
    if (pulse_delay1 > 10){
        pulse_delay1 = 0;
        if (led1) {
//            LED1 = 1;
            led1 = 0;
        } else {
//            LED1 = 0;
            led1 = 1;
        }
    }
    pulse_delay1++;
//    if (leftRotations > 450) leftSpeed = 0;
//    sprintf(str1,"RIGHT: %d || ",rightRotations);
//    sprintf(str2,"LEFT: %d\n\r",leftRotations);
//    uart_write_string(str1);
//    uart_write_string(str2);
}

void __ISR( _INPUT_CAPTURE_4_VECTOR, ipl1auto) IC4InterruptHandler (void){
    rightPeriod = IC4BUF;
    rightRotations = rightRotations + 1;
    TMR3 = 0;
    mIC4ClearIntFlag();
//    correctSpeeds();
    if (pulse_delay2 > 10){
        pulse_delay2 = 0;
        if (led2) {
//            LED2 = 1;
            led2 = 0;
        } else {
//            LED2 = 0;
            led2 = 1;
        }
    }
    pulse_delay2++;
//    if (rightRotations > 1000) rightSpeed = 0;
}

void setupIR(){
    analog_in_init();
}





uint32_t ir_right,ir_left,ir_front;
void readIRSensors(){
    ir_left = analog_in_read(IR_LEFT);
    ir_front = analog_in_read(IR_FRONT);
    ir_right = analog_in_read(IR_RIGHT);
    int i = 0;
    for (i = 0; i < SAMPLE_SIZE; i++){
        ir_right += analog_in_read(IR_RIGHT);
        ir_front += analog_in_read(IR_FRONT);
        ir_left += analog_in_read(IR_LEFT);
    }
    ir_right /= SAMPLE_SIZE;
    ir_left /= SAMPLE_SIZE;
    ir_front /= SAMPLE_SIZE;
    
    if(ir_left > IR_THRESH) {
        LED1 = 1;
    }
    else if(ir_right > IR_THRESH) {
        LED2 = 1;
    }
    else{ 
        LED1 = 0;
        LED2 = 0;
    }
}

void delayms(uint32_t ms){
    TMR4 = 0;
    int i = 0;
    for (i = 0; i < ms; i++){
        while(TMR4 != 0){}
    }
}

void turnRight(){
    sprintf(str1,"TURNING RIGHT\n\r");
    uart_write_string(str1);
    
    switch (currFace){
        case EAST: currFace = SOUTH; break;
        case WEST: currFace = NORTH; break;
        case NORTH: currFace = EAST; break;
        case SOUTH: currFace = WEST; break;
        default: printf("Some error while turning right");
    }
    
    rightSpeed = -MAX_SPEED;
    leftSpeed = MAX_SPEED;
    rightRotations = 0;
    leftRotations = 0;
    while((rightRotations < TURN_ROTATIONS) && (leftRotations < TURN_ROTATIONS)){
        
        if (rightRotations < TURN_ROTATIONS){
            rightSpeed = -MAX_SPEED;
        } else {
            rightSpeed = 0;
        }
        if (leftRotations < TURN_ROTATIONS){
            leftSpeed = MAX_SPEED;
        } else {
            leftSpeed = 0;
        }
    }
    rightSpeed = 0;
    leftSpeed = 0;
}

void turnLeft(){
    sprintf(str1,"TURNING LEFT\n\r");
    uart_write_string(str1);
    
    switch (currFace){
        case EAST: currFace = NORTH; break;
        case WEST: currFace = SOUTH; break;
        case NORTH: currFace = WEST; break;
        case SOUTH: currFace = EAST; break;
        default: printf("Some error while turning left");
    }
    
    rightSpeed = MAX_SPEED;
    leftSpeed = -MAX_SPEED;
    rightRotations = 0;
    leftRotations = 0;
    while((rightRotations < TURN_ROTATIONS) && (leftRotations < TURN_ROTATIONS)){
        if (rightRotations < TURN_ROTATIONS){
            rightSpeed = MAX_SPEED;
        } else {
            rightSpeed = 0;
        }
        if (leftRotations < TURN_ROTATIONS){
            leftSpeed = -MAX_SPEED;
        } else {
            leftSpeed = 0;
        }
    }
    rightSpeed = 0;
    leftSpeed = 0;
}

void moveForward(){
    switch (currFace){
        case EAST: currCell = currCell + 1; break;
        case WEST: currCell = currCell - 1; break;
        case NORTH: currCell = currCell - cols; break;
        case SOUTH: currCell = currCell + cols; break;
        default: printf("Some error while moving forward");
    }
    
    rightSpeed = MAX_SPEED;
    leftSpeed = MAX_SPEED;
    rightRotations = 0;
    leftRotations = 0;
    int rightBias = 0;
    int leftBias = 0;
    
    while((rightRotations < FORWARD_ROTATIONS+rightBias) && (leftRotations < FORWARD_ROTATIONS+leftBias-35)){
        readIRSensors();
//        sprintf(str1,"RIGHT: %d || ",ir_right);
//        sprintf(str2,"LEFT: %d || ",ir_left);
//        sprintf(str3,"FRONT: %d >>------> ",ir_front);
//        uart_write_string(str1);
//        uart_write_string(str2);
//        uart_write_string(str3);
        
        
        if (ir_right > 300) { //RIGHTSIDE IS TOO CLOSE TO WALL: SLOW LEFT MOTOR
//            float speedBias = 0.2*(1 - 1/(float)(ir_right - 360));
            float speedBias = 0.2*((float)(ir_right - 300)/400); //0.2*((float)(ir_right - 300)/500)
            rightBias = 20;
            leftSpeed = (int)((float)MAX_SPEED*(1-speedBias));
        }
        if (ir_left > 300) { //LEFTSIDE IS TOO CLOSE TO WALL: SLOW RIGHT MOTOR 
//            float speedBias = 0.2*(1 - 1/(float)(ir_left - 360));
            float speedBias = 0.2*((float)(ir_left - 300)/400);
            leftBias = 0;
            rightSpeed = (int)((float)MAX_SPEED*(1-speedBias));
        }
        
        
//        sprintf(str1,"RIGHTSPEED: %d || ",rightSpeed);
//        sprintf(str2,"LEFTSPEED: %d\n\r",leftSpeed);
//        uart_write_string(str1);
//        uart_write_string(str2);
        if (ir_front > 420) {  //450
            leftSpeed = 0;
            rightSpeed = 0;
            return;
        }
        if (rightRotations < FORWARD_ROTATIONS+rightBias){
            //maintain right speed
        } else {
            rightSpeed = 0;
        }
        if (leftRotations < FORWARD_ROTATIONS+leftBias-35){
            //maintain right speed
        } else {
            leftSpeed = 0;
        }
//        delayms(10);
//        sprintf(str1,"RIGHT: %d || ",rightRotations);
//        sprintf(str2,"LEFT: %d\n\r",leftRotations);
//        uart_write_string(str1);
//        uart_write_string(str2);
//        uart_write_nb((char)rightRotations);
//        uart_write8_nb((uint8_t) rightRotations);
//        uart_write8_nb((uint8_t) leftRotations);
    }
    rightSpeed = 0;
    leftSpeed = 0;
}

//////////////////////////////////////////////////////////////////
//turn to the direction of EAST
void turnEast(){
    switch (currFace){
        case EAST: break;
        case WEST: turnRight(); turnRight(); break;
        case NORTH: turnRight(); break;
        case SOUTH: turnLeft(); break;
        default: printf("Some error while turning");
    }
}
//turn to the direction of WEST
void turnWest(){
    switch (currFace){
        case EAST: turnRight(); turnRight(); break;
        case WEST: break;
        case NORTH: turnLeft(); break;
        case SOUTH: turnRight(); break;
        default: printf("Some error while turning");
    }
}
//turn to the direction of NORTH
void turnNorth(){
    switch (currFace){
        case EAST: turnLeft(); break;
        case WEST: turnRight(); break;
        case NORTH: break;
        case SOUTH: turnRight(); turnRight(); break;
        default: printf("Some error while turning");
    }
}
//turn to the direction of SOUTH
void turnSouth(){
    switch (currFace){
        case EAST: turnRight(); break;
        case WEST: turnLeft(); break;
        case NORTH: turnRight(); turnRight(); break;
        case SOUTH: break;
        default: printf("Some error while turning");
    }
}
//////////////////////////////////////////////////////////////////

/**
 * @brief moveToClosestUnexplored moves robot to the closest cell
 *      with an unexplored wall/direction
 */
void moveToClosestUnexplored(){
    // move to closest cell with an unexplored wall
    //currCell = pendingCells[pendingCntr-1];
    //pendingCntr--;

    //////////////////////////////////////////////////////////////
    // This finds the closest unexplored cell on the list of    //
    // cells of paths taken. It then traverses that list        //
    // creating an array of directions (basically just          //
    // reversing E/W and N/S, now that it's going               //
    // backward). These "directions" are then used to retrace   //
    // the path that was / taken from the closest unexplored    //
    //cell to the current cell, and changes the stepNum (index) // 
    //to the value of the cell it traversed back                //
    //////////////////////////////////////////////////////////////
//    printf("Finding closest unexplored\n");
    sprintf(str1,"Finding closest unexplored\n\r");
    uart_write_string(str1);
    int i=0;
    uint8_t directions[256];
    int newStepNum=-1;
    cell_type * cell = &cellContainer[currCell];

    for(i=0; i<stepNum; i++){
        if(pathTaken[i][0]==pendingCells[pendingCntr-1]){
	    pendingCntr--;
            newStepNum=i;
        } 
    }

    int numDirections=stepNum-newStepNum;

    int k = 0;
    for(i=stepNum-1; i>newStepNum-1; i--){
//	printf("Reverse directions at %d: %d\n",k,pathTaken[i][1]);
        if(pathTaken[i][1]==EAST){
            directions[k]=WEST;
        }else if(pathTaken[i][1]==WEST){
            directions[k]=EAST;
        }else if(pathTaken[i][1]==NORTH){
            directions[k]=SOUTH;
        }else if(pathTaken[i][1]==SOUTH){
            directions[k]=NORTH;
        }
        k++;        
    }

    stepNum=newStepNum;

    sprintf(str1,"Navigating to closest unexplored cell\n\r");
    uart_write_string(str1);
    for(i=0; i<numDirections; i++){
    sprintf(str1,"Current cell: %d\n\r",currCell);
    uart_write_string(str1);
//	printf("Current cell: %d\n",currCell);
        if(directions[i]==EAST){
//	    printf("Turning East\n");
            turnEast();
            delayms(STEP_DELAY);
            moveForward();
            delayms(STEP_DELAY);
        }else if(directions[i]==WEST){
//	    printf("Turning West\n");
            turnWest();
            delayms(STEP_DELAY);
            moveForward();
            delayms(STEP_DELAY);
        }else if(directions[i]==NORTH){
//	    printf("Turning North\n");
            turnNorth();
            delayms(STEP_DELAY);
            moveForward();
            delayms(STEP_DELAY);
        }else if(directions[i]==SOUTH){
//	    printf("Turning South\n");
            turnSouth();
            delayms(STEP_DELAY);
            moveForward();
            delayms(STEP_DELAY);
        }
    }
    //////////////////////////////////////////////////////////////

    //if (cellContainer[currCell].pending & EAST) currFace = EAST;
    //else if (cellContainer[currCell].pending & WEST) currFace = WEST;
    //else if (cellContainer[currCell].pending & NORTH) currFace = NORTH;
    //else if (cellContainer[currCell].pending & SOUTH) currFace = SOUTH;
}

/**
 * @brief mappingCompleted checks whether mapping is completed
 * @return 1 if completed, 0 otherwise
 */
uint8_t mappingCompleted(){
    int i = 0;
    for (i = 0; i < numberOfCells; i++){
        if (cellContainer[i].pending) return 0;
    }
    LED1 = 1;
    LED2 = 1;
    return 1;
}

/**
 * @brief step makes decision on where to move
 */
void step(){
    cell_type * cell = &cellContainer[currCell];

    while(!mappingCompleted()){
    // if in simulation mode, prompt the user to input cell type
    // on an actual robot, this will be done using physical sensors
    // ...besides for the first cell, the back wall should always be
    // marked as open (0)
#ifdef SIM
    int buf = 0;
    printf("Enter the orientation for cell %d\n", cell->index);
    scanf("%d", &buf);
    cell->orientation = (uint8_t)buf;
    printf("Entered orientation for cell %d: %d\n", cell->index, cell->orientation);
#endif
    delayms(STEP_DELAY);
    readIRSensors();
    printf("FRONT SENSOR: %d || RIGHT SENSOR: %d || LEFT SENSOR: %d\n",ir_front,ir_right,ir_left);
    if (ir_right > IR_THRESH){
        printf("Right wall detected\n");
        cell->orientation = cell->orientation | rightDir();
    } 
    if (ir_front > IR_FRONT_THRESH){
        printf("Front wall detected\n");
        cell->orientation = cell->orientation | currFace;
    } 
    if (ir_left > IR_THRESH){
        printf("Left wall detected\n");
        cell->orientation = cell->orientation | leftDir();
    } 

    // if we are starting to map and robot is on first grid,
    // number that cell zero and mark the back wall as explored
    if (!startedMapping){
        startedMapping = 1;
        cell->pending = cell->pending & (~backDir() & 0x0F);
        cell->pathValue = 0;
    }

    int16_t currPathVal = cell->pathValue;

    // update linked cells
    if ((~cell->orientation & 0x0F) & EAST) cell->linkedCells[0] = currCell + 1;
    if ((~cell->orientation & 0x0F) & WEST) cell->linkedCells[1] = currCell - 1;
    if ((~cell->orientation & 0x0F) & NORTH) cell->linkedCells[2] = currCell - rows;
    if ((~cell->orientation & 0x0F) & SOUTH) cell->linkedCells[3] = currCell + rows;

    sprintf(str1,"Linked Cells -> EAST:%d, WEST:%d, NORTH:%d, SOUTH:%d\n\r", cell->linkedCells[0], cell->linkedCells[1], cell->linkedCells[2], cell->linkedCells[3]);
    uart_write_string(str1);
//    printf("Linked Cells -> EAST:%d, WEST:%d, NORTH:%d, SOUTH:%d\n", cell->linkedCells[0], cell->linkedCells[1], cell->linkedCells[2], cell->linkedCells[3]);

    // if the robot can turn right and right wall is unexplored:
    // turn right,
    // mark the right wall as explored,
    // move forward one step
    // update the path value of the cell
    if (((~cell->orientation & 0x0F) & rightDir()) && (cell->pending & rightDir())) {
        // check surrounding walls
        if (!((~cell->orientation & 0x0F) & currFace)) cell->pending = cell->pending & (~currFace);
        if (!((~cell->orientation & 0x0F) & leftDir())) cell->pending = cell->pending & (~leftDir() & 0x0F);
        turnRight();
        delayms(STEP_DELAY);
        //////////////////////////////////////////////////////////
        pathTaken[stepNum][0]=cell->index;
        pathTaken[stepNum][1]=currFace;
        stepNum++;
        //////////////////////////////////////////////////////////

        cell->pending = cell->pending & (~currFace & 0x0F);
        // update closest pending index
        if (cell->pending) {
            pendingCells[pendingCntr] = currCell;
            pendingCntr++;
            lastKnownPendingIndex = cell->index;
        }
        moveForward();
        checkPathMapping(cellContainer[currCell],currPathVal);
        
        sprintf(str1,"Path value for cell %d: %d\n\r", currCell, cellContainer[currCell].pathValue);
        uart_write_string(str1);
//        printf("Path value for cell %d: %d\n", currCell, cellContainer[currCell].pathValue);
    } else if ((~cell->orientation & 0x0F) & currFace && (cell->pending & currFace)) {
        // if the robot can move forward, forward wall is unexplored and can't move right:
        // mark the front and right walls as explored,
        // move forward one step
        // update the path value of the cell
        cell->pending = cell->pending & (~rightDir() & 0x0F);
        cell->pending = cell->pending & (~currFace & 0x0F);
        // check surrounding walls
        if (!((~cell->orientation & 0x0F) & leftDir())) cell->pending = cell->pending & (~leftDir());
        // update closest pending index
        if (cell->pending) {
            pendingCells[pendingCntr] = currCell;
            pendingCntr++;
            lastKnownPendingIndex = cell->index;
        }
        moveForward();

        //////////////////////////////////////////////////////////
        pathTaken[stepNum][0]=cell->index;
        pathTaken[stepNum][1]=currFace;
        stepNum++;
        //////////////////////////////////////////////////////////

        checkPathMapping(cellContainer[currCell],currPathVal);
//        printf("Path value for cell %d: %d\n", currCell, cellContainer[currCell].pathValue);
        sprintf(str1,"Path value for cell %d: %d\n\r", currCell, cellContainer[currCell].pathValue);
        uart_write_string(str1);
    } else if ((~cell->orientation & 0x0F) & leftDir() && (cell->pending & leftDir())) {
        // if the robot can move left, left wall is unexplored and can't move right or forward:
        // mark the left, front and right walls as explored,
        // move forward one step
        // update the path value of the cell
        cell->pending = cell->pending & (~rightDir() & 0x0F);
        cell->pending = cell->pending & (~currFace & 0x0F);
        turnLeft();
        delayms(STEP_DELAY);
        //////////////////////////////////////////////////////////
        pathTaken[stepNum][0]=cell->index;
        pathTaken[stepNum][1]=currFace;
        stepNum++;
        //////////////////////////////////////////////////////////

        cell->pending = cell->pending & (~currFace & 0x0F);
        // update closest pending index
        if (cell->pending) {
            pendingCells[0] = currCell;
            pendingCntr++;
            lastKnownPendingIndex = cell->index;
        }
        moveForward();
        checkPathMapping(cellContainer[currCell],currPathVal);
        sprintf(str1,"Path value for cell %d: %d\n\r", currCell, cellContainer[currCell].pathValue);
        uart_write_string(str1);
//        printf("Path value for cell %d: %d\n", currCell, cellContainer[currCell].pathValue);
    } else {
        // if the robot can't move left, right or forward:
        // mark the left, front and right walls as explored,
        // turn around
        // move back to closest cell with an unexplored wall
        // update the path value of the cell
        cell->pending = 0;
        moveToClosestUnexplored();
        sprintf(str1,"Path value for cell %d: %d\n\r", currCell, cellContainer[currCell].pathValue);
        uart_write_string(str1);
//        printf("Path value for cell %d: %d\n", currCell, cellContainer[currCell].pathValue);
    }

    // for the cell that you wind up in, mark the back wall
    // as explored
    cell = &cellContainer[currCell];
    cell->pending = cell->pending & (~backDir() & 0x0F);
    }
    
//    uint8_t x, y;
//    for (x = 0; x < rows; x++){
//        for(y = 0; y < cols; y++){
//            printf("| %d |", y, cellContainer[y].pathValue);
//        }
//        printf("Cell %d: %d\n\r", x, cellContainer[x].pathValue);
//    }
}

/*
 * 
 */
int main() {
//    INTEnableInterrupts();
//    INTEnableSystemSingleVectoredInt();
    INTEnableSystemMultiVectoredInt();
//    INTEnableInterrupts();
//    INTCONSET = _INTCON_MVEC_MASK;
//    __builtin_enable_interrupts();
    
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    TRISA = 0x0000;
    TRISAbits.TRISA0 = 1;
    TRISAbits.TRISA1 = 1;
    TRISBbits.TRISB5 = 0;
    TRISC = 0x0000;
    TRISBbits.TRISB8 = 1;
    TRISBbits.TRISB3 = 1;
    RIGHT_MOTOR_EN = 1;
    LEFT_MOTOR_EN = 1;

    T1CON = 0x8010;
    T2CON = 0x8070;
    T3CON = 0x8070;
    T4CON = 0x8030;
    IC3CON = 0x8383;
    IC4CON = 0x8303;  
    IC3R = 0x4;
    IC4R = 0x1;
    
    TMR1 = 0;
    PR1 = 10;
    TMR2 = 0;
    PR2 = 0xFFFF;
    TMR3 = 0;
    PR3 = 0xFFFF;
    TMR4 = 0;
    PR4 = 5000;
    setupIR();
    uart_init();
    
    
    
    // Initialize interrupts
    // Set T1 interrupt source to have a priority of 1
    mT1SetIntPriority(1);
    mIC3SetIntPriority(1);
    mIC4SetIntPriority(1);
    // Turn on system interrupts
//    INTEnableSystemSingleVectoredInt();
//    INTEnableInterrupts();
    // Enable the T1 interrupt source
    mT1IntEnable(1);
    mIC3IntEnable(1);
    mIC4IntEnable(1);
//    moveForward();
//    LED1 = 0;
//    LED2 = 0;
//    rightSpeed = 0;
//    leftSpeed = 0;
    
//    uart_write_string("HELLO\n\r");
    while(1){
        LED1 = 0;
        LED2 = 0;
        rightSpeed = 0;
        leftSpeed = 0;
        printf("IDLE STATE...");
        
        while(SW1) { //wait to press SW1
            if(!SW2){ //makes a spin and flash state if SW2 is pressed before SW1
                printf("FLASH STATE...");
                while(1){
                    delayms(100);
                    LED1 = 0;
                    LED2 = 1;
                    delayms(100);
                    LED1 = 1;
                    LED2 = 0;
                    if(!SW1 || !SW2){
                        delayms(250);
                        break;
                    }
                }
            }
        }
        LED1 = 0;
        LED2 = 0;
        delayms(1000);
        printf("WANDER STATE ...");
        while(1){
            rows = 6;
            cols = 6;

            numberOfCells = rows * cols;

            // initialize all cells
            uint8_t i = 0;
            for (i = 0; i < numberOfCells; i++){
                cell_type cell;
                cell.orientation = 0;
                cell.index = i;
                cell.pathValue = -1;
                cell.pending = 15;
                cell.linkedCells[0] = 0;
                cell.linkedCells[1] = 0;
                cell.linkedCells[2] = 0;
                cell.linkedCells[3] = 0;
                cellContainer[i] = cell;
            }

            for (i = 0; i < numberOfCells; i++){
                pendingCells[i] = 0;
            }

            // start mapping
            printf("Starting Mapping Algorithm...\n");
            step();
            
            LED1 = 1;
            
            printf("Cell Value Summary...\n");
            for (i = 0; i < numberOfCells; i++){
                printf("Cell %d: %d\n", i, cellContainer[i].pathValue);
            }

        printf("Maze Successfully Solved :)\n");
        printf("Ready For Solve...");
        
        //        if(SW2){
        //            uint8_t x, y;
        //            for (x = 0; i < rows; i++){
        //                for(y = 0; i < cols; i++){
        //                    printf("| %d |", i, cellContainer[i].pathValue);
        //                }
        //                printf("Cell %d: %d\n\r", i, cellContainer[i].pathValue);
        //            }
        //        }
        }
    }
    return 1;
}

