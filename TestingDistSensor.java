package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link SensorREV2mDistance} illustrates how to use the REV Robotics
 * Time-of-Flight Range Sensor.
 *
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://revrobotics.com">REV Robotics Web Page</a>
 */
@TeleOp(name="Testing Rev2m Distance", group="Test and Development")

public class TestingDistSensor extends LinearOpMode {

    private DistanceSensor sensorLeft, sensorRight, sensorCenter;

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        sensorLeft = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        sensorRight = hardwareMap.get(DistanceSensor.class, "distanceRight");
        sensorCenter = hardwareMap.get(DistanceSensor.class, "distanceCenter");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor leftTimeOfFlight = (Rev2mDistanceSensor)sensorLeft;
        Rev2mDistanceSensor rightTimeOfFlight = (Rev2mDistanceSensor)sensorRight;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            // generic DistanceSensor methods.
            telemetry.addData("deviceName",sensorLeft.getDeviceName() );
            //telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", sensorLeft.getDistance(DistanceUnit.CM)));
            telemetry.addData("deviceName",sensorRight.getDeviceName() );
            telemetry.addData("range", String.format("%.01f cm", sensorRight.getDistance(DistanceUnit.CM)));
            telemetry.addData("deviceName",sensorCenter.getDeviceName() );
            telemetry.addData("range", String.format("%.01f cm", sensorCenter.getDistance(DistanceUnit.CM)));
            //telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));
            //telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));

            // Rev2mDistanceSensor specific methods.
            //telemetry.addData("ID", String.format("%x", rightTimeOfFlight.getModelID()));
            //telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            telemetry.update();
        }
    }

}