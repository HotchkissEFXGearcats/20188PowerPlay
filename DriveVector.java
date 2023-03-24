package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.sqrt;
import static java.lang.Math.atan2;
import static java.lang.Math.abs;

public class DriveVector {
    
    public double mag;
    public double angle;
    
    private double leftX, leftY, rightX, rightY;
    
    private double powerThreshold, precisionScalar, powerScalar;
    private double leftMag, rightMag;
    
    
    public void DriveVector() {
        this.mag = 0.0;
        this.angle = 0.0;
        this.leftMag = 0.0;
        this.rightMag = 0.0;
    }
    
    
    public void initialize() {
        this.powerThreshold = 0.05;
        this.precisionScalar = 0.2;
        this.powerScalar = 0.60;
    }
    
    
    
    // This methods corrects the Euler angle to a range of -PI to +PI with a shift
    // transformation to math the IMU heading.
    private void heading() {
        if ((angle < -PI/2) && (angle >= -PI)) {
            angle = 2*PI + angle - PI/2;
        } else {
            angle = angle - PI/2;
        }  // end if-else
        //return angle;
    }  // end method heading
    
    
    // This method accepts the left and right stick values from the gamepad, then it calculates either a 
    // a power or precision vector depending on priority (precision priority). It then calls the heading()
    // method to correct the Euler angle to -PI to +PI.
    public DriveVector makeVector(double leftStickX, double leftStickY, double rightStickX, double rightStickY) {
        
        //precisionScalar = 0.2;
        //powerScalar = 0.75;
        
        leftX = leftStickX;
        leftY = -leftStickY;
        rightX = rightStickX;
        rightY = -rightStickY;
        
        leftMag = sqrt((leftY * leftY) + (leftX * leftX));
        rightMag = sqrt((rightY * rightY) + (rightX * rightX));
        
        if ((abs(rightMag) > powerThreshold) || (abs(leftMag) < 0.01)) {
            mag = this.precisionScalar * rightMag;
            angle = atan2(rightY, rightX);
        } else {
            mag = this.powerScalar * leftMag;
            angle = atan2(leftY, leftX);
        }  // end if statement
        heading();
        return this;
    }  // end method makeVector

}  // end class


