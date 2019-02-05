package ev3Project;

import java.util.Timer;
import java.util.TimerTask;

import lejos.*;
import lejos.ev3.tools.LCDDisplay;
import lejos.hardware.BrickFinder;
import lejos.hardware.Keys;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

public class LineFollow1
{
    
    
    
    static EV3LargeRegulatedMotor LEFT_MOTOR;
    static EV3LargeRegulatedMotor RIGHT_MOTOR;
    
    
    
    long millis = System.currentTimeMillis();
    
    

    public static void main(String[] args)
    {
        //prepare motors/chassis
        EV3 ev3brick = (EV3) BrickFinder.getLocal(); 
        Keys buttons = ev3brick.getKeys(); 

        
        LEFT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.B);
        RIGHT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.C);
        
        Wheel wheel1= WheeledChassis.modelWheel(LEFT_MOTOR, 5.5).offset(-9);
        Wheel wheel2= WheeledChassis.modelWheel(RIGHT_MOTOR, 5.5).offset(9);    
        Chassis chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2}, WheeledChassis.TYPE_DIFFERENTIAL);
        MovePilot pilot = new MovePilot(chassis);
        
        
        
      //prepare colour sensor1
        EV3ColorSensor colourSensor1 = new EV3ColorSensor(SensorPort.S1);
        float[] sample1 = new float[3];//colourSensor1.sampleSize()]; 
        float leftSample;
        colourSensor1.setCurrentMode("RGB");        
        
      //prepare colour sensor2
        EV3ColorSensor colourSensor2 = new EV3ColorSensor(SensorPort.S2);
        float[] sample2 = new float[3];//colourSensor2.sampleSize()];       
        float rightSample;
        colourSensor2.setCurrentMode("RGB");        
        
        LCD.drawString("Initalisation complete", 0, 0);
        Delay.msDelay(500);
        
        
        long rotateStartTimeL = 0;
        long rotateStartTimeR = 0;
        int delay = 600;

        
        
        while(buttons.getButtons() != Keys.ID_ESCAPE) {
            Delay.msDelay(30);
            colourSensor1.fetchSample(sample1, 0);
            leftSample = sample1[0];
            colourSensor2.fetchSample(sample2, 0);
            rightSample = sample2[0];
            LCD.clear();              
            LCD.drawString("Left: "+sample1[0], 0, 0);  
            LCD.drawString("Right: "+sample2[0], 0, 5);  
            
          /*
            LEFT_MOTOR.forward();
            RIGHT_MOTOR.forward();
            String currentAction = "";
            
            //RIGHT
            //White: 0.12 to 0.135
            //Black: 0.013 to 0.016
            //Line: 0.09 to 0.11
             
            //LEFT
            //White: 0.117 to 0.125
            //Black: 0.013 to 0.016
            //Line: 0.09 to 0.11
          */
            
            //CONSTANTS
            float leftWhiteMin = 0.09f;//0.108f;
            float rightWhiteMin = 0.09f;//0.114f;       
            
            float leftLineMin = 0.09f;
            float rightLineMin = 0.09f;
            
            float leftBlackMax = 0.025f;//0.017f;
            float rightBlackMax = 0.029f;//0.021f;
            
            /*
            if(leftSample < leftBlackMax && rightSample > rightWhiteMin) {                  //Left sensor on black, right sensor on white
                rotateLeft();
            }else if(rightSample < rightBlackMax && leftSample > leftWhiteMin) {             //Left sensor on white, right sensor on black
                rotateRight();            
            }else if(leftSample < leftWhiteMin && leftSample > leftBlackMax && rightSample > rightBlackMax) {        //Left sensor between black/white, right sensor on white or between black/white
                turnLeft();
            }else if(rightSample < rightWhiteMin && rightSample > rightBlackMax && leftSample > leftBlackMax) {        //Right sensor between black/white, left sensor on white or between black/white
                turnRight();
            }else {
                medForward();
            }
            
            */
            
            long currentTime = System.currentTimeMillis();
          
            /*
            if(leftSample < leftBlackMax && rightSample < rightBlackMax) {                  //Left sensor on black, right sensor on black
                medForward();           
            }else
            */
                
            if(leftSample < leftBlackMax && rightSample > rightBlackMax && currentTime > (rotateStartTimeR+delay)) {                  //Left sensor on black
                rotateLeft();
                //pilot.rotate(-20);
                rotateStartTimeL = System.currentTimeMillis();
            }else if(rightSample < rightBlackMax && leftSample > leftBlackMax && currentTime > (rotateStartTimeL+delay)) {             //Right sensor on black
                rotateRight(); 
                //pilot.rotate(20);
                rotateStartTimeR = System.currentTimeMillis();
            }else {
                medForward();
            }

            
            
            
            
        }
        

        
        
        
        
        /*
        colourInput[0] = -1;
        while (colourInput[0] > 0.5) {
            colourSensor.fetchSample(colourInput, 0); //getRed(colourSensor, colourInput);
            LCD.clear(); 
            float x = colourInput[0]; 
            LCD.drawString("Sample: " + x, 0, 0);
            pilot.forward();
            
        }
        
        LCD.drawString("Complete", 0, 0);
        Delay.msDelay(800);
        */
        
        
        System.exit(0);

    }
    
    
    public static void fastForward() {
        RIGHT_MOTOR.setSpeed(180);
        LEFT_MOTOR.setSpeed(180);
        LEFT_MOTOR.forward();
        RIGHT_MOTOR.forward();
    }
    
    public static void medForward() {
        RIGHT_MOTOR.setSpeed(110);
        LEFT_MOTOR.setSpeed(110);
        LEFT_MOTOR.forward();
        RIGHT_MOTOR.forward();
    }
    
    public static void slowForward() {
        RIGHT_MOTOR.setSpeed(65);
        LEFT_MOTOR.setSpeed(65);
        LEFT_MOTOR.forward();
        RIGHT_MOTOR.forward();
    }
    
    public static void turnLeft() {
        LEFT_MOTOR.setSpeed(50);
        RIGHT_MOTOR.setSpeed(200);
        LEFT_MOTOR.backward();
        RIGHT_MOTOR.forward();
    }
    
    public static void turnRight() {
        LEFT_MOTOR.setSpeed(200);
        RIGHT_MOTOR.setSpeed(50);
        LEFT_MOTOR.forward();
        RIGHT_MOTOR.backward();
    }

    public static void rotateLeft() {
        LEFT_MOTOR.stop();
        RIGHT_MOTOR.stop();
        //pilot.rotate(10);

        
        LEFT_MOTOR.setSpeed(400);
        RIGHT_MOTOR.setSpeed(300);
        LEFT_MOTOR.backward();
        RIGHT_MOTOR.forward();
        
        Delay.msDelay(10);
    }
    
    public static void rotateRight() {
        LEFT_MOTOR.stop();
        RIGHT_MOTOR.stop();
        //pilot.rotate(-10);
        
        LEFT_MOTOR.setSpeed(300);
        RIGHT_MOTOR.setSpeed(400);
        LEFT_MOTOR.forward();
        RIGHT_MOTOR.backward();
        
        Delay.msDelay(10);
    }
    
    
    
    
    /*
    public static void setRedMode(EV3ColorSensor colourSensor)
    {
        colourSensor.setCurrentMode("Red");
    }
    
    
    public float getRed(EV3ColorSensor colourSensor, float[] colourInput)
    {
        colourSensor.fetchSample(colourInput, 0);        
        return colourInput[0];
    }
    */
}
