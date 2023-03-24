package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ConeThread extends Thread {
    
    private HardwareMap hardwareMap;
    private LinearOpMode opModeTool;
    
    private SensorNetwork sensors;
    private NormalizedRGBA cone;
    
    public void ConeThread(HardwareMap hardwareMap, NormalizedRGBA cone) {
        this.hardwareMap = hardwareMap;
        this.cone = cone;
    }

    public void run() {
        
        sensors = new SensorNetwork(hardwareMap, opModeTool);
        sensors.initialize(7, 7, 7);
        
        cone = sensors.getForwardColors();
        while ((cone.alpha < 0.1) && true) {   // put in opmodeactive
            cone = sensors.getForwardColors();
            opModeTool.idle();
        }
        
        
        
        
        
        
        
    }  // end run   
    
}  // end class