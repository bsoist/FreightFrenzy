package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

//parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

@Autonomous(name = "OpenCVTest")
public class opencv_autotest extends LinearOpMode {
    OpenCvCamera Camera;
    @Override
    public void runOpMode() throws InterruptedException {
    int cameraMonitorViewId = hardwareMap.appContext
            .getResources()
            .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    WebcamName Webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

    OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(Webcam, cameraMonitorViewId);

    opencv_test detector = new opencv_test(telemetry);

    camera.setPipeline(detector);
    camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
    {
        @Override
        public void onOpened()
        {
            // Usually this is where you'll want to start streaming from the camera (see section 4)
            camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        }
        @Override
        public void onError(int errorCode)
        {
            /*
            * This will be called if the camera could not be opened
            */
        }
    });

    waitForStart();
        switch (detector.getLocation()) {
            case LEFT:
                // ...
                break;
            case RIGHT:
                // ...
                break;
            case NOT_FOUND:
                // ...
        }
        Camera.stopStreaming();
    }
}
