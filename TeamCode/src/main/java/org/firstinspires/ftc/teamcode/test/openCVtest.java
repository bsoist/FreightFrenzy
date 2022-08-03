
package org.firstinspires.ftc.teamcode.test;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

@Autonomous(name = "OpenCVTest")
//@Disabled
public class openCVtest extends LinearOpMode {
    OpenCvCamera camera;
    openCVTeamMarkerDetection pipeline;
    openCVTeamMarkerDetection.Location snapshotAnalysis = openCVTeamMarkerDetection.Location.RIGHT; // default
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new openCVTeamMarkerDetection();
        camera.setPipeline(pipeline);

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
        switch (pipeline.getLocation()) {
            case LEFT:
//                botLevel = true;
                break;
            case MIDDLE:
//                midLevel = true;
                break;
            case RIGHT:
//                topLevel = true;
                break;
            case NOT_FOUND:
//                topLevel = true;
        }
        camera.stopStreaming();
    }
}