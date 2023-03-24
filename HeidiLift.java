package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.hardware.TouchSensor;

import static java.lang.Math.abs;

// BEGIN CLASS //
public class HeidiLift extends Thread {
    
    /* CLASS MEMBERS */
    
    private HardwareMap hardwareMap;
    private LinearOpMode opModeTool;
    
    private DcMotor motor;
    private int position, zero, max, threshold, buffer, bottomBuffer, topBuffer;
    private int currentPos, highPos, midPos, lowPos, hoverPos, poleBuffer;
    private double fast, slow;
    
     public HeidiLift(HardwareMap hardwareMap, LinearOpMode opModeTool) {
        this.hardwareMap = hardwareMap;
        this.opModeTool = opModeTool;
       // initialize();
    } // end constructor
    
    public int initialize(double lowSpeed, double highSpeed) {
        
        motor = hardwareMap.get(DcMotor.class, "motorLift");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        max = 3500;
        
        bottomBuffer = 400;
        topBuffer = 400;
        
        highPos = 3445;
        midPos = 2460;
        lowPos = 1490;
        hoverPos = 370;
        poleBuffer = 200;
        
        zero = -motor.getCurrentPosition();
        threshold = zero + bottomBuffer;
        buffer = max - topBuffer;
        position = -motor.getCurrentPosition();
        
        fast = highSpeed;
        slow = lowSpeed;
        
        //return zero;
        return -99;
        
    } // end method initialize
    
    
    public int getPosition() {
        return position;
    }
    
    
    public int moveManual(double power) {
        if (power < 0.0) {
            motor.setPower(slow * power);
        } else if (power > 0.0) {
            motor.setPower(slow * power);
        } else {
            motor.setPower(0.0);
        }
        return -motor.getCurrentPosition();
    }  // end method moveTesting
    
    
    
    public int move(double power) {
    if (power < 0.0) {
        up(power);
    } else if (power > 0.0) {
        down(power);
    } else {
        motor.setPower(0.0);
    }
        return -motor.getCurrentPosition();
    }  // end method move

    

    private void up(double power) {
        if (-motor.getCurrentPosition() < max) {
            if (-motor.getCurrentPosition() < buffer) {
                motor.setPower(fast * power);
            } else {
                motor.setPower(slow * power);
            }
        } else {
            motor.setPower(0.0);
        }  // end if-else
    }  // end method up
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

    //  ****** ADD PICKUP SPEED DOWN TO THIS ???? ******* //
    
    /* METHOD DOWN */
    private void down(double power) {
        if (-motor.getCurrentPosition() > zero) {
            if (-motor.getCurrentPosition() > threshold) {
                motor.setPower(fast * power);
            } else {
                motor.setPower(slow * power);
            }
        } else {
            motor.setPower(0.0);
        }  // end if-else
    }  // end method down














// ****** MAKE THE SIGNAL METHODS BI-DIRECTIONAL ****** //


public int topSignal() {
    currentPos = -motor.getCurrentPosition();
    while (currentPos < (highPos)) {
        if (currentPos < (highPos - poleBuffer)) {
            motor.setPower(-fast); // change this to fast
        } else {
            motor.setPower(-slow);
        }
        currentPos = -motor.getCurrentPosition();
    }  // end while
    motor.setPower(0.0);
    return -motor.getCurrentPosition();
}  // end method topSignal



public int midSignal() {
    currentPos = -motor.getCurrentPosition();
    while (currentPos < (midPos)) {
        if (currentPos < (midPos - poleBuffer)) {
            motor.setPower(-fast); // change this to fast
        } else {
            motor.setPower(-slow);
        }
        currentPos = -motor.getCurrentPosition();
    }  // end while
    motor.setPower(0.0);
    return -motor.getCurrentPosition();
}  // end method midSignal



/*
*  This method moves the lift up to the low signal pole
*  deposit height. It only moves up. If the lift is above
*  the low signal pole already, it will not move.
*/
public int lowSignal() {
    currentPos = -motor.getCurrentPosition();
    while (currentPos < lowPos) {
        if (currentPos < (lowPos - poleBuffer)) {
            motor.setPower(-fast); // change this to fast
        } else {
            motor.setPower(-slow);
        }
        currentPos = -motor.getCurrentPosition();
    }  // end while
    motor.setPower(0.0);
    return -motor.getCurrentPosition();
}  // end method lowSignal















public int hover() {
    currentPos = -motor.getCurrentPosition();
    if (currentPos < hoverPos) {
        while (currentPos < hoverPos) {
            motor.setPower(-slow);
            currentPos = -motor.getCurrentPosition();
        }  // end while 
    } else if (currentPos > hoverPos) {
        while (currentPos > hoverPos) {
            if (currentPos > (hoverPos + poleBuffer)) {
                motor.setPower(fast);
            } else {
                motor.setPower(slow);
            }
            currentPos = -motor.getCurrentPosition();
        }  // end while 
    }
    motor.setPower(0.0);
    return -motor.getCurrentPosition();
}  // end method hover



public int resetToZero() {
    currentPos = -motor.getCurrentPosition();
    if (currentPos > 0) {
        while (currentPos >0) {
            if (currentPos > 100) {
                motor.setPower(fast); // change this to fast
            } else {
                motor.setPower(slow);
            }
            currentPos = -motor.getCurrentPosition();
        }  // end while 
    }
    motor.setPower(0.0);
    return -motor.getCurrentPosition();
}



public void run() {
    if (opModeTool.gamepad2.y) {
        topSignal();
    } else if (opModeTool.gamepad2.x) {
        midSignal();
    } else if (opModeTool.gamepad2.a) {
        hover();
    } else if (opModeTool.gamepad2.b) {
        lowSignal();
    }
}  // end method run



}  // end class


