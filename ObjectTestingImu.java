package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.lang.Math.abs;
import static java.lang.Math.sin;
import static java.lang.Math.cos;
import static java.lang.Math.PI;

public class ObjectTestingImu {
    
    /* CLASS MEMBERS */
    
    HardwareMap hardwareMap;
    LinearOpMode opModeTool;
    
    //private HeidiLinOpModeTool opModeTool;
    
    private IMU imu;
    private YawPitchRollAngles orientation;
    private AngularVelocity angularVelocity;
    
    private double heading, previous, duration;
    private boolean flag;
    
    ElapsedTime timer;
    
    /* CLASS CONSTRUCTOR */
    public ObjectTestingImu(HardwareMap hardwareMap, LinearOpMode opModeTool) {
        this.hardwareMap = hardwareMap;
        this.opModeTool = opModeTool;
        heading = 0.0;
        previous = 0.0;
        duration = -9.0;
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        flag = true;
    }  // end constructor
    
    
    
    /* CLASS METHODS */
    
    public int initialize() {
        // Insantiate and initialize IMU using Parameters
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        
        angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        orientation = imu.getRobotYawPitchRollAngles();
        heading = orientation.getYaw(AngleUnit.RADIANS);
        
        return 1;
    }  // end method initialize
    
    
    // version 2
    public double updateHeading() {
        previous = heading;
        orientation = imu.getRobotYawPitchRollAngles();
        //opModeTool.idle();
        heading = orientation.getYaw(AngleUnit.RADIANS);
        timer.reset();
        while (abs(heading - previous) < 0.1) {
            opModeTool.idle();
            heading = orientation.getYaw(AngleUnit.RADIANS);
            if ((timer.milliseconds() > 30.0) || !opModeTool.opModeIsActive()) {break;}
        }  // end while
        return timer.milliseconds();
    }  // end method updateHeading
    
    
    
    public double updateHeadingOnce() {
        if (flag) {
            
            previous = heading;
            orientation = imu.getRobotYawPitchRollAngles();
            //opModeTool.idle();
            heading = orientation.getYaw(AngleUnit.RADIANS);
            duration = 0.0;
            timer.reset();
            while (abs(heading - previous) < 0.1) {
                opModeTool.idle();
                heading = orientation.getYaw(AngleUnit.RADIANS);
                duration = timer.milliseconds();
                if ((duration > 30.0) || !opModeTool.opModeIsActive()) {break;}
            }  // end while
        }  // end if
        flag = false;
        return duration;
    }  // end method updateHeading
    
    public void resetFlag() {
        flag = true;
        duration = -1.0;
    }
    
    
    
    
    /* version 1
    public int updateHeading() {
        orientation = imu.getRobotYawPitchRollAngles();
        opModeTool.idle();
        heading = orientation.getYaw(AngleUnit.RADIANS);
        return 1;
    } */
    
    public double heading() {
        return heading;
    }  // end method runImu
    
    public void idle() {
        opModeTool.idle();
    }
    
    
    
    public double checkObjIdle() {
        timer.reset();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        return timer.milliseconds();
    }
    
    
    /*
    public int updateHeading() {
        previous = heading;
        orientation = imu.getRobotYawPitchRollAngles();
        opModeTool.idle();
        heading = orientation.getYaw(AngleUnit.RADIANS);
        if (abs(heading - previous) < 0.01) {
            loops = 0;
            while((abs(heading - previous) < 0.01) && (loops < 1000)) {
                loops += 1;
                opModeTool.idle();
            }
        }  // end if
        return loops;
    }
    */
    
}  // end class


