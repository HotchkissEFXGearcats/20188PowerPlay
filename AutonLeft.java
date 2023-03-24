package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;
import static java.lang.Math.PI;

@Autonomous(name="Auton LEFT", group="Auton")

public class AutonLeft extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public int spot;
    private DriveVector vector;
    private HeidiDrive drive;
    private HeidiLift lift;
    private Intake intake;
    private SensorNetwork sensors;
    private NormalizedRGBA cone;
    private ElapsedTime timer;

    @Override
    public void runOpMode() {
        
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        
        drive = new HeidiDrive(hardwareMap, this);
        drive.initialize();
        
        vector = new DriveVector();
        vector.initialize();
        
        sensors = new SensorNetwork(hardwareMap, this);
        sensors.initialize(7, 7, 7);
        
        lift = new HeidiLift(hardwareMap, this);
        lift.initialize(0.15, 0.5);
        
        intake = new Intake(hardwareMap, this);
        intake.initialize();
        
        spot = 0;
        
        cone = sensors.getForwardColors();
        
        telemetry.addData("Status", "Initialized");
        telemetry.addLine();
        telemetry.addData("Red   : ", "%.3f", cone.red);
        telemetry.addData("Green : ", "%.3f", cone.green);
        telemetry.addData("Blue  : ", "%.3f", cone.blue);
        telemetry.update();
        
        waitForStart();
        runtime.reset();
        
        intake.in(500);
        lift.lowSignal();
        
        vector.mag = 0.2;
        vector.angle = 0.0;
        drive.autonVector(vector, 1550);
        drive.stop();
        
        cone = sensors.getForwardColors();
        
        timer.reset();
        
        while ((cone.alpha < 0.1)) {           // && opModeIsActive
            cone = sensors.getForwardColors();
            if (timer.milliseconds() > 2000.0) {break;}   // || !opModeTool.opModeIsActive()
        }
        
        if ((cone.blue > cone.green) && (cone.blue > cone.red)) {
            //1
            spot = 1;
        } else if ((cone.red > cone.green) && (cone.red > cone.blue)) {
            //2 
            spot = 2;
        } else if ((cone.green >= cone.red) && (cone.green >= cone.blue)) {
            //3 
            spot = 3;
        } 
        
        telemetry.addData("Status", "Color Detection");
        telemetry.addLine();
        telemetry.addData("Alpha  : ", "%.3f", cone.alpha);
        telemetry.addLine();
        telemetry.addData("Red   : ", "%.3f", cone.red);
        telemetry.addData("Green : ", "%.3f", cone.green);
        telemetry.addData("Blue  : ", "%.3f", cone.blue);
        telemetry.addLine();
        telemetry.addData("Zone  : ", spot);
        telemetry.update();
        
        drive.turnTo(0.1, -0.62);
        drive.linearPosition(true);
        
        lift.midSignal();
        
        vector.mag = 0.2;
        vector.angle = -0.62;
        drive.autonVector(vector, 680);
        drive.stop();
        drive.linearPosition(true);
        
        intake.out(2000);
        
        vector.mag = 0.2;
        vector.angle = -0.62 - PI;
        drive.autonVector(vector, 680);
        drive.stop();
        lift.hover();
        
        if (spot == 1) {
            // Park in 1
            drive.turnTo(-0.1, PI/2.0);
            drive.stop();
            drive.linearPosition(true);
            vector.mag = 0.2;
            vector.angle = PI/4.0;
            drive.autonVector(vector, 200);
            drive.stop();
            drive.linearPosition(true);
            vector.mag = 0.2;
            vector.angle = PI/2.0;
            drive.autonVector(vector, 1100);
            drive.stop();
            drive.turnTo(0.1, 0.0);
            lift.resetToZero();
        } else if (spot == 3) {
            // Park in 3
            drive.turnTo(0.1, -PI/2.0);
            drive.stop();
            drive.linearPosition(true);
            vector.mag = 0.2;
            vector.angle = -PI/4.0;
            drive.autonVector(vector, 200);
            drive.stop();
            drive.linearPosition(true);
            vector.mag = 0.2;
            vector.angle = -PI/2.0;
            drive.autonVector(vector, 1100);
            drive.stop();
            drive.turnTo(-0.1, 0.0);
            lift.resetToZero();
        } else {
            // Park in 2 by default
            drive.turnTo(-0.1, 0.0);
            drive.stop();
            drive.linearPosition(true);
            vector.mag = 0.2;
            vector.angle = 0.0;
            drive.autonVector(vector, 200);
            drive.stop();
            lift.resetToZero();
        }
        
        
        // Show the elapsed game time 
        telemetry.addData("Spot: ", spot);
        telemetry.addLine();
        telemetry.update();
            
    }  // end method runopmode
    
    void goVector(double mag, double angle, int encoder){
        vector.mag=mag;
        vector.angle=angle;
        drive.autonVector(vector, encoder);
        drive.stop();
        drive.linearPosition(true);
    }
    
    void turn(double power, double toHeading){
        drive.turnTo(power, toHeading);
        drive.linearPosition(true);
    }
    
}  // end class

