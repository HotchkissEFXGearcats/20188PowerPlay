package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class WhatColor {
    
    HardwareMap hardwareMap;
    NormalizedColorSensor colorSensor;
    
    
    public WhatColor(HardwareMap hardwareMap){
        this.hardwareMap=hardwareMap;
        initialize();
    }
    
    public void initialize(){
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorR");
            float gain = 5;
            colorSensor.setGain(gain);
    }
    
    int getColor(){
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
            return (0);
        }
        else if (Math.abs(red-blueRed)<=marginError && Math.abs(green-blueGreen)<=marginError && Math.abs(blue-blueBlue)<=marginError){
            return (-1);
        }
        else if (Math.abs(red-redRed)<=marginError && Math.abs(green-redGreen)<=marginError && Math.abs(blue-redBlue)<=marginError){
            return (1);
        }
        return(0);

    }
    // todo: write your code here
}