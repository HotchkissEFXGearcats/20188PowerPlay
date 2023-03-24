/*
 * COMPETITION TELEOP FOR H-BOT 
 * 
 * VERSION: Jan. 29
 *
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
@TeleOp(name="HEIDI TELEOP", group="Heidi Opmodes")

public class HeideCompTeleop extends LinearOpMode {

    // Declare OpMode members.\
    private ElapsedTime runtime = new ElapsedTime();
    //DriveVector vectorM;
   
    @Override
    public void runOpMode() {
        
        HeidiDrive drive = new HeidiDrive(hardwareMap, this);
        drive.initialize();
        
        DriveVector vector = new DriveVector();
        vector.initialize();
        //DriveVector vectorDebug = new DriveVector();
        
        float leftTurn = 0;
        float rightTurn = 0;
        
        int position = 0;
        
        double loops = 0.0;
        
        //double heading = 0.0;
        Headings heading = new Headings();
        
        telemetry.addData("Status", "Initialized:  B");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)\
        while (opModeIsActive()) {
            
            leftTurn = gamepad1.left_trigger;
            rightTurn = gamepad1.right_trigger;
            
            //vectorDebug = vector.makeVector(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
            vector.makeVector(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
            drive.goVector(vector, (rightTurn - leftTurn));
            
            if (gamepad1.b) {
                loops = drive.updateHeadingOnce();
            }
            if (gamepad1.x) {
                drive.resetFlag();
            }
            
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addLine();
            telemetry.addData("Control Pad Left Stick: ", "left X (%.2f), left Y (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("Control Pad Sticks: ", "right Y (%.2f), right Y (%.2f)", gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("Triggers, ", "Left: (%.2f), Right: (%.2f)",  leftTurn, rightTurn);
            telemetry.addLine();
            //telemetry.addData("Debug Vector, Mag: ", "(%.2f)", vectorDebug.mag);
            telemetry.addData("Drive Vector, Mag: ", "(%.2f)", vector.mag);
            telemetry.addData("Drive Vector, Angle: ", "(%.2f)", vector.angle);
            telemetry.addLine();
            telemetry.addData("driveHeading : ", "%.6f", vector.angle);
            telemetry.addData("botHeading   : ", "%.6f", heading.botHeading);
            telemetry.addData("setHeading   : ", "%.6f", heading.setHeading);
            telemetry.addLine();
            telemetry.addData("Loops Time : ", "%.6f", loops);
            telemetry.addLine();
            telemetry.update();
        }  // end while loop
        
    }  // end method runOpMode
 
    
    
}  // end class
