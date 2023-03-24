package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

//import com.qualcomm.robotcore.hardware.Blinker;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Gyroscope;
//import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import static java.lang.Math.abs;
import static java.lang.Math.sin;
import static java.lang.Math.cos;
import static java.lang.Math.PI;


// BEGIN CLASS //

public class HeidiDrive {
    
    private HardwareMap hardwareMap;
    private LinearOpMode opModeTool;
    
    private ElapsedTime timer;
    
//    private Gyroscope imu;
    
    private IMU imu;
    private DcMotor leftMotor, rightMotor, centerMotor;
    private int k;
    private int positionL, positionR, positionC, avgPosition;
    private int start, finish, getGoing, slowDown, driveBuffer;
    private double setHeading, driveHeading, botHeading, headingOffset, previous, duration;
    
    private YawPitchRollAngles orientation;
    private AngularVelocity angularVelocity;
    
    private double kp, kAuto;
    private double scaleTurn;
    private double turn;
    private boolean flag, warning;
    private boolean wasTurning;
    
    private Headings headings; 
    
    /* CLASS CONSTRUCTOR */
    public HeidiDrive(HardwareMap hardwareMap, LinearOpMode opModeTool) {
        this.hardwareMap = hardwareMap;
        this.opModeTool = opModeTool;
       // initialize();
    }
    
// @TeleOp(name="HeidiDrive", group="Classes")    
    
    public void initialize() {
        
        //
        // Initialize IMU using Parameters
        //
        
        imu = hardwareMap.get(IMU.class, "imu");
        
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        
        
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        
        angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        orientation = imu.getRobotYawPitchRollAngles();
        setHeading = orientation.getYaw(AngleUnit.RADIANS);
        
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        
        headings = new Headings();
        headings.botHeading = botHeading;
        headings.setHeading = setHeading;
        
        //
        // Initialize Motors
        //
        
        leftMotor = hardwareMap.get(DcMotor.class, "motorL");
        rightMotor = hardwareMap.get(DcMotor.class, "motorR");
        centerMotor = hardwareMap.get(DcMotor.class, "motorC");
        
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        positionL = leftMotor.getCurrentPosition();
        positionR = rightMotor.getCurrentPosition();
        positionC = centerMotor.getCurrentPosition();
        
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        centerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        centerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        //
        // Initialize scalars and flags
        //
       
        flag = true;
        warning = false;
        
        driveHeading = 0.0;
        botHeading = 0.0;
        headingOffset = 0.0;
        previous = 0.0;
        duration = 0.0;
        
        kp = 0.35;
        kAuto = 0.4;
        scaleTurn = 0.25;
        wasTurning = false;
        flag = true;
        
        driveBuffer = 300;
        
    }  // end method initialize
    
    
    
    public boolean autonVector(DriveVector vector, int toPosition) {
        if (warning || (vector.mag < 0.05)) {
            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);
            centerMotor.setPower(0.0);
        } else {
            while (abs(linearPosition(false)) < toPosition) {
                orientation = imu.getRobotYawPitchRollAngles();
                opModeTool.idle();
                botHeading = orientation.getYaw(AngleUnit.RADIANS);
                headingOffset = (botHeading - setHeading);
                if (abs(headingOffset) < PI/4) {
                    turn = (kAuto * headingOffset) / (PI/4);
                } else if (abs(headingOffset) < PI/3) {
                    turn = kAuto * 1;
                } else {
                    leftMotor.setPower(0.0);
                    rightMotor.setPower(0.0);
                    centerMotor.setPower(0.0);
                    warning = true;
                }  // end if-else turn < 0.05 case is not turning
                leftMotor.setPower(vector.mag * sin((vector.angle - setHeading + PI/2)) + turn );  //yPower
                rightMotor.setPower(vector.mag * sin((vector.angle - setHeading + PI/2)) - turn );  //yPower
                centerMotor.setPower(vector.mag * cos((vector.angle - setHeading + PI/2)));  //xPower
                opModeTool.telemetry.addData("Position: ", linearPosition(false));
                opModeTool.telemetry.update();
            }  // end while
        }  // end if-else
        return warning;
    }  // end method autonVector
        
    
    public int linearPosition(boolean reset) {
        
        if (reset) {
            
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            centerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            centerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            centerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
        } else {
            
            positionL = leftMotor.getCurrentPosition();
            positionR = rightMotor.getCurrentPosition();
            //positionC = centerMotor.getCurrentPosition();
            
            avgPosition = (int)((positionL + positionR)/2);
            
        } // end if-else
        
        return avgPosition;
        
    }  // end method drivePosition
    
    
    // 
    // Stop the bot
    //
    
    public int stop() {
        
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        centerMotor.setPower(0.0);
        
        positionL = leftMotor.getCurrentPosition();
        positionR = rightMotor.getCurrentPosition();
        //positionC = centerMotor.getCurrentPosition();
            
        avgPosition = (int)((positionL + positionR)/2);
        return avgPosition;
    }
    
    
    // 
    // Turn to heading
    //
    
    public double turnTo(double turnPower, double toHeading) {
        orientation = imu.getRobotYawPitchRollAngles();
        opModeTool.idle();
        botHeading = orientation.getYaw(AngleUnit.RADIANS);
        while (abs(botHeading - toHeading) > (PI/180)) {
            leftMotor.setPower(turnPower);
            rightMotor.setPower(-turnPower);
            orientation = imu.getRobotYawPitchRollAngles();
            opModeTool.idle();
            botHeading = orientation.getYaw(AngleUnit.RADIANS);
        }  // end while
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        orientation = imu.getRobotYawPitchRollAngles();
        opModeTool.idle();
        setHeading = orientation.getYaw(AngleUnit.RADIANS);
        return setHeading;
    }  // end method turn 
    
    
    
    
    
    
    
    
    
    
    /** DRIVER CONTROL BELOW **/
    
    
    public void goVector(DriveVector vector, double turnPower) {
        if ((vector.mag < 0.05)) {
            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);
            centerMotor.setPower(0.0);
            orientation = imu.getRobotYawPitchRollAngles();
            opModeTool.idle();
            setHeading = orientation.getYaw(AngleUnit.RADIANS);
        } else {
           
            orientation = imu.getRobotYawPitchRollAngles();
            opModeTool.idle();
            botHeading = orientation.getYaw(AngleUnit.RADIANS);
            headingOffset = (botHeading - setHeading);
            if (abs(headingOffset) < PI/4) {
                turn = (kAuto * headingOffset) / (PI/4);
            } else if (abs(headingOffset) < PI/3) {
                turn = kAuto * 1;
            } else {
                leftMotor.setPower(0.0);
                rightMotor.setPower(0.0);
                centerMotor.setPower(0.0);
            }  // end if-else turn < 0.05 case is not turning
            leftMotor.setPower(vector.mag * sin((vector.angle - setHeading + PI/2)) + turn +turnPower*0.2 );  //yPower
            rightMotor.setPower(vector.mag * sin((vector.angle - setHeading + PI/2)) - turn +turnPower*0.2);  //yPower
            centerMotor.setPower(vector.mag * cos((vector.angle - setHeading + PI/2)));  //xPower
            opModeTool.telemetry.addData("Position: ", linearPosition(false));
            opModeTool.telemetry.update();
        }  // end if-else
        
        
        
        
        
        
    }  // end method goVector
    
    
    public double updateHeadingOnce() {
        if (flag) {
            previous = setHeading;
            orientation = imu.getRobotYawPitchRollAngles();
            //opModeTool.idle();
            setHeading = orientation.getYaw(AngleUnit.RADIANS);
            duration = 0.0;
            timer.reset();
            while (abs(setHeading - previous) < 0.1) {
                opModeTool.idle();
                setHeading = orientation.getYaw(AngleUnit.RADIANS);
                duration = timer.milliseconds();
                if ((duration > 30.0) || !opModeTool.opModeIsActive()) {break;}
            }  // end while
        }  // end if
        flag = false;
        headings.setHeading = setHeading;
        return duration;
    }  // end method updateHeading
    
    public void resetFlag() {
        flag = true;
        duration = -1.0;
    }
    

    
    /* GOVECTOR METHOD */
    
    public void goVector2(DriveVector vector, double turnPower) {
        
        
        if (vector.mag < 0.05) {
        
            if (abs(turnPower) < 0.05) {
                leftMotor.setPower(0.0);
                        rightMotor.setPower(0.0);
                        centerMotor.setPower(0.0);
                 orientation = imu.getRobotYawPitchRollAngles();
                        opModeTool.idle();
                         setHeading = orientation.getYaw(AngleUnit.RADIANS);
            } else {
                leftMotor.setPower(turnPower * scaleTurn );
                        rightMotor.setPower(-turnPower * scaleTurn );
                orientation = imu.getRobotYawPitchRollAngles();
                        opModeTool.idle();
                         setHeading = orientation.getYaw(AngleUnit.RADIANS);
            }
        
        } else {
            
            if (abs(turnPower) < 0.05) {
        
        
                orientation = imu.getRobotYawPitchRollAngles();
                            opModeTool.idle();
                            botHeading = orientation.getYaw(AngleUnit.RADIANS);
                            headingOffset = (botHeading - setHeading);
                            if (abs(headingOffset) < PI/4) {
                                    turn = (kp * headingOffset) / (PI/4);
                            } else if (abs(headingOffset) < PI/3) {
                                    turn = kp * 1;
                            } else {
                                    turn = 0;
                            }  // end if-else
        
                            leftMotor.setPower(vector.mag * sin((vector.angle - setHeading + PI/2)) + turn );  //yPower
                            rightMotor.setPower(vector.mag * sin((vector.angle - setHeading + PI/2)) - turn );  //yPower
                            centerMotor.setPower(vector.mag * cos((vector.angle - setHeading + PI/2)));  //xPower
        
            } else {
                // drive and turn at the same time
                turn = turnPower * scaleTurn;
        
                leftMotor.setPower(vector.mag * sin((vector.angle - setHeading + PI/2)) + turn );  //yPower
                            rightMotor.setPower(vector.mag * sin((vector.angle - setHeading + PI/2)) - turn );  //yPower
                            centerMotor.setPower(vector.mag * cos((vector.angle - setHeading + PI/2)));  //xPower
        
                //wasTurning = true;
        
                orientation = imu.getRobotYawPitchRollAngles();
                       opModeTool.idle();
                       setHeading = orientation.getYaw(AngleUnit.RADIANS);
        
            }
        
            //If (wasTurning) {
        
            //    orientation = imu.getRobotYawPitchRollAngles();
                   //    opModeTool.idle();
                   //    setHeading = orientation.getYaw(AngleUnit.RADIANS);
            //    wasTurning = false;
        
            //}
        
        }
    }  // end method goVector2
    
    
    /* a turn only method, primarily used for testing turn rate */
    public double turn(double turnPower, double toHeading) {
        orientation = imu.getRobotYawPitchRollAngles();
        //opModeTool.idle();
        botHeading = orientation.getYaw(AngleUnit.RADIANS);
        while (abs(botHeading - toHeading + PI/2) > (PI/180)) {
            leftMotor.setPower(turnPower);
            rightMotor.setPower(-turnPower);
            orientation = imu.getRobotYawPitchRollAngles();
            opModeTool.idle();
            botHeading = orientation.getYaw(AngleUnit.RADIANS);
        }  // end while
        //leftMotor.setPower((botHeading - toHeading + PI/2)/(PI));  //yPower
        //rightMotor.setPower(-(botHeading - toHeading + PI/2)/(PI));  //yPower
        return botHeading;
    }  // end method turn 
        

}  // end class
