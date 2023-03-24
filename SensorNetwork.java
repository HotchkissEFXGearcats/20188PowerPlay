package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SensorNetwork {
    
    private HardwareMap hardwareMap;
    private DistanceSensor leftDistance, rightDistance, centerDistance;
    private NormalizedColorSensor forwardColor, frontColor, rearColor;
    private NormalizedRGBA forwardColors, frontColors, rearColors;
    private LinearOpMode opModeTool;
    
    /* CLASS CONSTRUCTOR */
    public SensorNetwork(HardwareMap hardwareMap, LinearOpMode opModeTool) {
        this.hardwareMap = hardwareMap;
        this.opModeTool = opModeTool;
    } 
    
    public void initialize(float gainForward, float gainFront, float gainRear) {
        leftDistance = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        rightDistance = hardwareMap.get(DistanceSensor.class, "distanceRight");
        centerDistance = hardwareMap.get(DistanceSensor.class, "distanceCenter");
        
        Rev2mDistanceSensor leftTimeOfFlight = (Rev2mDistanceSensor)leftDistance;
        Rev2mDistanceSensor rightTimeOfFlight = (Rev2mDistanceSensor)rightDistance;
        Rev2mDistanceSensor centerTimeOfFlight = (Rev2mDistanceSensor)centerDistance;
        
        forwardColor = hardwareMap.get(NormalizedColorSensor.class, "colorForward");
        frontColor = hardwareMap.get(NormalizedColorSensor.class, "colorFront");
        rearColor = hardwareMap.get(NormalizedColorSensor.class, "colorRear");
        
        forwardColor.setGain(gainForward);
        frontColor.setGain(gainFront);
        rearColor.setGain(gainRear);
        
    }
    
    public NormalizedRGBA getForwardColors() {
        forwardColors = forwardColor.getNormalizedColors();
        opModeTool.idle();
        return forwardColors;
    }
    
    public void getColors() {
        forwardColors = forwardColor.getNormalizedColors();
        frontColors = frontColor.getNormalizedColors();
        rearColors = rearColor.getNormalizedColors();
    }
    
}  // end class