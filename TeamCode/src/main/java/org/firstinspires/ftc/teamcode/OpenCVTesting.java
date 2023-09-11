package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.CvType;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

@TeleOp (name = "OpenCVTesting" ,group = "Linear Opmode")

//@Disabled
public class OpenCVTesting extends LinearOpMode {
    //Dashboard variables
    private FtcDashboard dashboard;

    //Vision variables
    OpenCvWebcam webcam;
    private Random rng = new Random(12345);

    //Motor demo variables
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRigtDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;

    private void initializeMotors()
    {
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        frontRigtDrive = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "backRightDrive");

        frontRigtDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRigtDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRigtDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initializeDashboard()
    {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

     private void initializeVision()
    {
         /* Instantiate an OpenCvCamera object for the camera we'll be using.
            * In this sample, we're using a webcam. Note that you will need to
            * make sure you have added the webcam to your configuration file and
            * adjusted the name here to match what you named it in said config file.
            *
         * We pass it the view that we wish to use for camera monitor (on
            * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        webcam.setPipeline(new OurOpenCVPipeline());
        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

    }


    public void runOpMode()  {
        initializeDashboard();
        initializeMotors();
        initializeVision();
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());

            telemetry.addData("Ball X", RobotConstants.ballX);
            telemetry.addData("Ball Y", RobotConstants.ballY);
            telemetry.addData("Ball Found", RobotConstants.ballExists);
            updateTelemetry(telemetry);

            double rotatePower = RobotConstants.ballX * RobotConstants.ROTATE_FACTOR;

            frontLeftDrive.setPower(-rotatePower);
            frontRigtDrive.setPower(rotatePower);
            backLeftDrive.setPower(-rotatePower);
            backRightDrive.setPower(rotatePower);
        }
    }


    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class OurOpenCVPipeline extends OpenCvPipeline {

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input) {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            boolean foundBall = false;

            //Convert to HSV for better color range definition
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
            //Filter pixels outside the desired color range
            Scalar color_lower = new Scalar(RobotConstants.color_lower_h, RobotConstants.color_lower_s, RobotConstants.color_lower_v);
            Scalar color_upper = new Scalar(RobotConstants.color_upper_h, RobotConstants.color_upper_s, RobotConstants.color_upper_v);
            Core.inRange(input, color_lower, color_upper, input);
            //De-speckle the image
            Mat element = Imgproc.getStructuringElement(RobotConstants.elementType,
                                                        new Size(2 * RobotConstants.kernelSize + 1,
                                                                 2 * RobotConstants.kernelSize + 1),
                                                        new Point(RobotConstants.kernelSize,
                                                                  RobotConstants.kernelSize));
            Imgproc.erode(input, input, element);
            //Find blobs in the image. Actually finds a list of contours which will need to be processed later
            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            final Mat hierarchy = new Mat();
            Imgproc.findContours(input, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            //Now search the list for the largest "blob"
            double largest_area = 0.0;
            int largest_contour_index = -1;
            for( int i = 0; i< contours.size(); i++ ) {// iterate through each contour.
                double a=Imgproc.contourArea( contours.get(i),false);  //  Find the area of contour
                if((a>largest_area) && (a>RobotConstants.minBallArea)){
                    foundBall = true;
                    largest_area=a;
                    largest_contour_index=i;                //Store the index of largest contour
                    //bounding_rect=boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
                }
            }
            //Draw the contours
            MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];
            Point[] centers = new Point[contours.size()];
            float[][] radius = new float[contours.size()][1];
            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                centers[i] = new Point();
                Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);
            }
            List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
            for (MatOfPoint2f poly : contoursPoly) {
                contoursPolyList.add(new MatOfPoint(poly.toArray()));
            }
            //Convert the image back to RGB for color drawing
            Imgproc.cvtColor(input, input, Imgproc.COLOR_GRAY2RGB);
            int R;
            int G;
            int B;
             for (int i = 0; i < contours.size(); i++) {
                 if (i == largest_contour_index) {
                     R = rng.nextInt(256);
                     G = rng.nextInt(256);
                     B = rng.nextInt(256);
                 }
                 else {
                     R = 255;
                     G = 0;
                     B = 0;
                 }
                 Scalar color = new Scalar(R, G, B);
                if (RobotConstants.drawContours == 1) {
                    Imgproc.drawContours(input, contoursPolyList, i, color);
                }
                if (RobotConstants.drawRectangle == 1) {
                    Imgproc.rectangle(input, boundRect[i].tl(), boundRect[i].br(), color, 2);
                }
                if (RobotConstants.drawCircle == 1) {
                    Imgproc.circle(input, centers[i], (int) radius[i][0], color, 2);
                }
            }

            RobotConstants.ballExists = foundBall;

             if (foundBall) {
                 RobotConstants.ballX = centers[largest_contour_index].x - (input.width() / 2);
                 RobotConstants.ballY = centers[largest_contour_index].y - (input.height() / 2);
             }

             /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }

    }
}

