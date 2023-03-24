/*
*  This opmode is for only test the IMU object formed from the ObjectTestingImu
*  class. It is to test read times and delay or check approaches to knowing 
*  when IMU Yaw data has been read.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Testing IMU", group="Test and Development")

public class TestingImu extends LinearOpMode {
    
    /* CLASS MEMBERS */
    ElapsedTime runtime;
    private double start, finish;

    @Override
    public void runOpMode() {
        
        runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        
        ObjectTestingImu imuObj = new ObjectTestingImu(hardwareMap, this);
        int init = 0;
        init = imuObj.initialize();
        
        double loops = 0.0;

        telemetry.addData("Status", "Initialized: rev B");
        telemetry.addData("IMU Init Status", init);
        telemetry.addData("Heading: ", "(%.6f)", imuObj.heading());
        telemetry.update();
        
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            /* runtime.reset();
            imuObj.idle();
            finish = runtime.milliseconds(); */
            
            if (gamepad2.b) {
                //loops = imuObj.updateHeading();
                loops = imuObj.updateHeadingOnce();
            }
            if (gamepad2.x) {
                imuObj.resetFlag();
            }
            
            telemetry.addData("Status", "Running");
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Time Duration, 5 Idle: ", finish);
            telemetry.addLine();
            telemetry.addData("Heading    : ", "%.6f", imuObj.heading());
            telemetry.addData("Loops Time : ", "%.15f", loops);
            //telemetry.addData("Obj Timer : ", "(%.6f)", imuObj.checkObjIdle());
            telemetry.update();

        }  // end opmode while
        
    }  // end method runOpMode
}  // end of class



