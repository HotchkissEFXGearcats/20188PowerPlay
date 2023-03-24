package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Intake {
    
    private HardwareMap hardwareMap;
    private LinearOpMode opModeTool;
    private ElapsedTime timer;
    
    private Servo roller;

    /* CLASS CONSTRUCTOR */
    public Intake(HardwareMap hardwareMap, LinearOpMode opModeTool) {
        this.hardwareMap = hardwareMap;
        this.opModeTool = opModeTool;
    }
    
    public void initialize() {
        
        roller = hardwareMap.get(Servo.class, "roller");
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        
    }  // end method initialize
    
    
    public void in(long howLong) {
        roller.setPosition(0);
        delay(howLong);
        roller.setPosition(0.5);
    }
    
    public void in() {
        roller.setPosition(0);
    }
    
    public void out(long howLong) {
        roller.setPosition(1);
        delay(howLong);
        roller.setPosition(0.5);
    }
    
    public void out() {
        roller.setPosition(1);
    }
    
    public void stop() {
        roller.setPosition(0.5);
    }
    
    
    //sleep for ms milliseconds
    private void delay(long ms)
    {
        try {
            TimeUnit.MILLISECONDS.sleep(ms);
        }
        catch(InterruptedException e) {
            
        }
    }
    
    
    
    
}  // end class


