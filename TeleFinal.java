package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="DRIVER", group="Linear Opmode")

public class TeleFinal extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime;
    private DriveVector vector;
    private HeidiDrive drive;
    private HeidiLift lift;
    private Intake intake;
    private double liftPower;
    private double leftTurn;
    private double rightTurn;

    @Override
    public void runOpMode() {
        
        runtime = new ElapsedTime();

        drive = new HeidiDrive(hardwareMap, this);
        drive.initialize();
        
        vector = new DriveVector();
        vector.initialize();
        
        intake = new Intake(hardwareMap, this);
        intake.initialize();
        
        lift = new HeidiLift(hardwareMap, this);
        lift.initialize(0.15, 0.95);
        lift.start();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            //lift
            
            liftPower = gamepad2.right_stick_y;
            lift.move(liftPower);
            
            if (gamepad2.right_bumper) {
                lift.moveManual(gamepad2.left_stick_y * 0.5);
            }
            //lift.move(0.0);
            
            /*
            if (gamepad2.y) {
                lift.topSignal();
            } else if (gamepad2.x) {
                lift.midSignal();
            } else if (gamepad2.a) {
                lift.hover();
            } else if (gamepad2.b) {
                lift.lowSignal();
            }*/
            
            //intake
            if (gamepad2.right_trigger > 0.1){
                intake.in();
            }
            else if (gamepad2.left_trigger > 0.1){
                intake.out();
            }
            else{
                intake.stop();
            }
            
            //drive
            leftTurn = gamepad1.left_trigger;
            rightTurn = gamepad1.right_trigger;
            
            //vectorDebug = vector.makeVector(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
            vector.makeVector(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
            drive.goVector2(vector, (rightTurn - leftTurn));
            
            //if (gamepad1.b) {
               // drive.updateHeadingOnce();
            //}
            
            

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    } // end method 
}  // end class
