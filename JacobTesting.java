/*
 * Jacogb testing enviornment
 * 
 * VERSION: Jan. 19
 *
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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

@TeleOp(name="Jacob Testing", group="Heidi Auton Testing")

public class JacobTesting extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Servo roller;
    
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor centerMotor;
    private DistanceSensor distanceR;
    private DistanceSensor distanceL;
    NormalizedColorSensor colorSensor;
   
    @Override
    public void runOpMode() {
        
        servoSetup();
        
        //HeidiDrive drive = new HeidiDrive(hardwareMap);
        //drive.initialize();
        
        //DriveVector vector = new DriveVector();
        
        leftMotor = hardwareMap.get(DcMotor.class, "motorL");
        rightMotor = hardwareMap.get(DcMotor.class, "motorR");
        centerMotor = hardwareMap.get(DcMotor.class, "motorC");
        distanceR = hardwareMap.get(DistanceSensor.class, "distanceR");
        distanceL = hardwareMap.get(DistanceSensor.class, "distanceL");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorR");
        

        
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
        
        Rev2mDistanceSensor sensorTimeOfFlight1 = (Rev2mDistanceSensor)distanceR;
        Rev2mDistanceSensor sensorTimeOfFlight2 = (Rev2mDistanceSensor)distanceL;
        
        float leftTurn = 0;
        float rightTurn = 0;
        int position;
        double heading = 0.0;
        
        HeidiLift lift = new HeidiLift(hardwareMap, this);
        position = lift.initialize(0.15, 0.95);
        int height = 0;
        double liftPower = 0.0;
        
        double lowDrive = 0.2;
        double highDrive = 0.6;
        double powerThreshold = 0.1;
        
        double leftX = 0.0;
        double leftY = 0.0;
        double rightX = 0.0;
        double rightY = 0.0;
        double turn = 0.0;
        
        double leftMag = 0.0;
        double rightMag = 0.0;
        
        float gain = 5;
        
        telemetry.addData("Status", "Initialized:  f");
        telemetry.addLine();
        telemetry.addData("Lift Position: ", position);
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
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
            
           
            
            // ucomment this to re-enable the lift:  liftPower = gamepad2.right_stick_y;
            //height = lift.move(liftPower);
            
            /* comment this out when not using for precision move */
            
            /* uncomment this to re-enable lift
            if (gamepad2.right_bumper) {
                height = lift.moveManual(liftPower);
            } else {
                height = lift.move(liftPower);
            }
            */
            
            /*if (gamepad1.a) {
                // Only increase the gain by a small amount, since this loop will occur multiple times per second.
                gain += 0.005;
              } else if (gamepad1.b && gain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
                gain -= 0.005;
              }
              
             colorSensor.setGain(gain);*/

      // Show the gain value via telemetry
      colorSensor.setGain(gain);
      telemetry.addData("Gain", gain);
            
            telemetry.addData("right distance", String.format("%.01f mm", distanceR.getDistance(DistanceUnit.MM)));
            telemetry.addData("left distance", String.format("%.01f mm", distanceL.getDistance(DistanceUnit.MM)));
            //telemetry.addData("Lift Power: ", liftPower);
            //telemetry.addData("Stick Y: ", gamepad2.right_stick_y);
            //telemetry.addLine();
            //telemetry.addData("IMU Heading:   ", heading);
            //telemetry.addData("Driver Vector: ", vector.angle);
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
        
              telemetry.addLine()
                      .addData("Red", "%.3f", colors.red)
                      .addData("Green", "%.3f", colors.green)
                      .addData("Blue", "%.3f", colors.blue);
              //telemetry.addData("Alpha", "%.3f", colors.alpha);
        
              /* If this color sensor also has a distance sensor, display the measured distance.
               * Note that the reported distance is only useful at very close range, and is impacted by
               * ambient light and surface reflectivity. */
              /*if (colorSensor instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
              }*/
              telemetry.addData("Color", getColor());
        
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
    
    String getColor(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double marginError=0.006;
        double red=colors.red;
        double green=colors.green;
        double blue=colors.blue;
        double greyRed=0.010;
        double greyGreen=0.019;
        double greyBlue=0.019;
        double blueRed=0.007;
        double blueGreen=0.017;
        double blueBlue=0.042;
        double redRed=0.021;
        double redGreen=0.014;
        double redBlue=0.010;
        
        if (Math.abs(red-greyRed)<=marginError && Math.abs(green-greyGreen)<=marginError && Math.abs(blue-greyBlue)<=marginError){
            return ("grey");
        }
        else if (Math.abs(red-blueRed)<=marginError && Math.abs(green-blueGreen)<=marginError && Math.abs(blue-blueBlue)<=marginError){
            return ("blue");
        }
        else if (Math.abs(red-redRed)<=marginError && Math.abs(green-redGreen)<=marginError && Math.abs(blue-redBlue)<=marginError){
            return ("red");
        }
        return("grey");

    }

    public DriveVector makeVector(double leftStickX, double leftStickY, double rightStickX, double rightStickY) {
        
        double leftX = leftStickX;
        double leftY = -leftStickY;
        double rightX = rightStickX;
        double rightY = -rightStickY;
        
        DriveVector vectorM = new DriveVector();
        
        double powerThreshold = 0.1;
        double precisionScalar = 0.2;
        double powerScalar = 0.5;
        
        double leftMag = sqrt((leftY * leftY) + (leftX * leftX));
        double rightMag = sqrt((rightY * rightY) + (rightX * rightX));
        
        if ((abs(rightMag) > powerThreshold) || (abs(leftMag) < 0.01)) {
            vectorM.mag = precisionScalar * rightMag;
            vectorM.angle = atan2(rightY, rightX);
        } else {
            vectorM.mag = powerScalar * leftMag;
            vectorM.angle = atan2(leftY, leftX);
        }  // end if statement
        return vectorM;
    }  // end method makeVector
    
    
}  // end class
