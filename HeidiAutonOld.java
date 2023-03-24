package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Core;
import org.opencv.imgproc.Moments;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.ArrayList;

@Autonomous

public class HeidiAutonOld extends LinearOpMode{
    private DcMotor left, right, middle;
    private DcMotor lift;
    
    OpenCvWebcam webcam;
    
    @Override
    public void runOpMode() {
        motorSetup();
        webcamSetup();
        
        waitForStart();
    }
    
    //sleep for ms milliseconds
    public static void delay(long ms)
    {
        try {
            TimeUnit.MILLISECONDS.sleep(ms);
        }
        catch(InterruptedException e) {
            
        }
    }

    void motorSetup() {
        right = hardwareMap.get(DcMotor.class, "motorR");
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setDirection(DcMotor.Direction.REVERSE);
        
        left = hardwareMap.get(DcMotor.class, "motorL");
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left.setDirection(DcMotor.Direction.REVERSE);
        
        middle = hardwareMap.get(DcMotor.class, "motorC");
        middle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        lift = hardwareMap.get(DcMotor.class, "motorLift");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    
    void webcamSetup() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        
        webcam.setPipeline(new heidiColor());

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                
            }
        });
    }
}

class heidiColor extends OpenCvPipeline {
    public static int maxColor = 0;
    
    Mat blurredImage = new Mat();
    Mat hsvImage = new Mat();
    
    Mat thresh = new Mat();
    
    Mat output = new Mat();
    
    Mat dilated = new Mat();
    Mat hierarchey = new Mat();
    List<MatOfPoint> contours = new ArrayList<>();
    
    private double colorProcess(Mat input, Scalar minColor, Scalar maxColor) {
        contours.clear();

        output = input;
        
        Imgproc.blur(input, blurredImage, new Size(3, 3));
        Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_BGR2HSV);
        
        Core.inRange(hsvImage, minColor, maxColor, thresh);     
        
        //return colorRange;
        
        //dilate the image, to make it better for the next step
        //make the blobs bigger
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.dilate(thresh, dilated, kernel);
        
        //find the contours of the image
        Imgproc.findContours(dilated, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(input, contours, -1, new Scalar(0,255,0), 1);
        
        double maxVal = 0;
        int maxValIdx = 0;
        if (!contours.isEmpty()) {
            for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++)
            {
                double contourArea = Imgproc.contourArea(contours.get(contourIdx));
                if (maxVal < contourArea)
                {
                    maxVal = contourArea;
                    maxValIdx = contourIdx;
                }
            }
        } 
        
        return maxVal;
    }
    
    double purple;
    double green;
    double orange;
    
    @Override 
    public Mat processFrame(Mat input)
    {
        purple = colorProcess(input, new Scalar(0, 146, 43), new Scalar(189, 194, 89));
        orange = colorProcess(input, new Scalar(94, 195, 121), new Scalar(103, 255, 153));
        green = colorProcess(input, new Scalar(66, 160, 77), new Scalar(81, 214, 141));
        
        if (orange > purple && orange > green) {
            maxColor = 1;
        } 
        if (purple > orange && purple > green) {
            maxColor = 2;
        }
        if (green > purple && green > orange) {
            maxColor = 3;
        }
        
        return input;
    }
}
