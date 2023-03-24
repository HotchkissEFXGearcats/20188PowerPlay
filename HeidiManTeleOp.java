/*
 * COMPETITION TELEOP FOR H-BOT 
 * 
 * VERSION: Jan. 19
 *
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;
import static java.lang.Math.atan;
import static java.lang.Math.atan2;
import static java.lang.Math.PI;

// BEGIN CLASS //

@TeleOp(name="HEIDI MANUAL TELEOP", group="Heidi Opmodes")

public class HeidiManTeleOp extends LinearOpMode {
    
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Servo roller;
    
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor centerMotor;
    
    private IMU imu;
    private YawPitchRollAngles orientation;
    private AngularVelocity angularVelocity;
    
    private double heading;
   
    @Override
    public void runOpMode() {
        
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        
        angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        orientation = imu.getRobotYawPitchRollAngles();
        heading = orientation.getYaw(AngleUnit.RADIANS);
        
        servoSetup();
        
        leftMotor = hardwareMap.get(DcMotor.class, "motorL");
        rightMotor = hardwareMap.get(DcMotor.class, "motorR");
        centerMotor = hardwareMap.get(DcMotor.class, "motorC");
        
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
//        positionL = leftMotor.getCurrentPosition();
//        positionR = rightMotor.getCurrentPosition();
//        positionC = centerMotor.getCurrentPosition();
        
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        centerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        centerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        float leftTurn = 0;
        float rightTurn = 0;
        int position;
        double heading = 0.0;
        
        HeidiLift lift = new HeidiLift(hardwareMap, this);
        position = lift.initialize(0.15, 0.95);
        int height = 0;
        double liftPower = 0.0;
        
        double lowDrive = 0.2;
        double highDrive = 0.7;
        double powerThreshold = 0.1;
        
        double leftX = 0.0;
        double leftY = 0.0;
        double rightX = 0.0;
        double rightY = 0.0;
        double turn = 0.0;
        
        double leftMag = 0.0;
        double rightMag = 0.0;
        
        telemetry.addData("Status", "Initialized:  f");
        telemetry.addLine();
        telemetry.addData("Lift Position: ", position);
        telemetry.addLine();
        telemetry.addData("Heading: ", "(%.15f)", heading);
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            orientation = imu.getRobotYawPitchRollAngles();
            heading = orientation.getYaw(AngleUnit.RADIANS);
            
            rollerPower();
            leftTurn = gamepad1.left_trigger;
            rightTurn = gamepad1.right_trigger;
            turn = 0.25 * (rightTurn - leftTurn);
            
            leftX = gamepad1.left_stick_x;
            leftY = -gamepad1.left_stick_y;
            rightX = gamepad1.right_stick_x;
            rightY = -gamepad1.right_stick_y;
            
            leftMag = sqrt((leftY * leftY) + (leftX * leftX));
            rightMag = sqrt((rightY * rightY) + (rightX * rightX));
            
            if ((abs(rightMag) > powerThreshold) || (abs(leftMag) < 0.01)) {
                leftMotor.setPower(lowDrive * rightY + turn);
                rightMotor.setPower(lowDrive * rightY - turn);
                centerMotor.setPower(lowDrive * rightX);
            } else {
                leftMotor.setPower(highDrive * leftY + turn);
                rightMotor.setPower(highDrive * leftY - turn);
                centerMotor.setPower(highDrive * leftX);
            }
            

            /* Master lift driver block */
            
            liftPower = gamepad2.right_stick_y;
            lift.move(liftPower);
            //lift.moveManual(gamepad2.left_stick_y*0.8);
            
            
            if (gamepad2.y) {
                lift.topSignal();
            } else if (gamepad2.x) {
                lift.midSignal();
            } else if (gamepad2.a) {
                lift.hover();
            } else if (gamepad2.b) {
                lift.lowSignal();
            }
            
            if (gamepad2.right_bumper) {
                height = lift.moveManual(liftPower);
            } else {
                height = lift.move(liftPower);
            }
            
            
            //lift.moveManual(gamepad2.left_stick_y * 0.8);
            
            
            
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addLine();
            telemetry.addData("Lift Position: ", height);
            telemetry.addData("Liftpower from Stick: ", "(%.2f)", liftPower);
            telemetry.addLine();
            //telemetry.addData("Gamepad A: ", gamepad2.a);
            //telemetry.addData("Gamepad B: ", gamepad2.b);
            //telemetry.addData("Gamepad X: ", gamepad2.x);
            //telemetry.addData("Gamepad Y: ", gamepad2.y);
            //telemetry.addData("Lift Power: ", liftPower);
            //telemetry.addData("Stick Y: ", gamepad2.right_stick_y);
            telemetry.addLine();
            telemetry.addData("IMU Heading:   ", "(%.15f)", heading);
            telemetry.update();
            
            /* Drivetrain Telemetry 
            telemetry.addData("Control Pad Left Stick: ", "left X (%.2f), left Y (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("Control Pad Sticks: ", "right Y (%.2f), right Y (%.2f)", gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addLine();
            telemetry.addData("Motor", "power : (%.2f)", vector.mag);
            telemetry.addData("Drive Angle: ", "(%.2f)", vector.angle);
            telemetry.addLine();
            telemetry.addData("Heading: ", "(%.2f)", heading);
            telemetry.addLine();
            telemetry.addData("atan(0,0): ", "(%.2f)", atan2(0.0, 0.0));
            telemetry.addLine();
            telemetry.addData("Triggers, ", "Left: (%.2f), Right: (%.2f)",  leftTurn, rightTurn);
            telemetry.update();
            
            */
            
        }  // end while loop
        
    }  // end method runOpMode
    
    
    void servoSetup(){
        roller = hardwareMap.get(Servo.class, "roller");
    }
    
    void rollerPower(){
        if (gamepad2.right_trigger > 0.1){
            roller.setDirection(Servo.Direction.REVERSE);
            roller.setPosition(1);
        }
        else if (gamepad2.left_trigger > 0.1){
            roller.setDirection(Servo.Direction.FORWARD);
            roller.setPosition(1);
        }
        else{
            roller.setPosition(0.5);
        }
    }

    
}  // end class
