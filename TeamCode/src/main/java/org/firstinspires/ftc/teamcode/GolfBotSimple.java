package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

@TeleOp (name = "GolfBotSimple" ,group = "Linear Opmode")

//@Disabled
public class GolfBotSimple extends LinearOpMode {
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
    private DcMotorEx clubMotor = null;
    private DistanceSensor ballDistance = null;
    private RevBlinkinLedDriver ledLights = null;

    private BNO055IMU imu = null;      // Control/Expansion Hub IMU

    private double robotHeading  = 0;
    private double headingOffset = 0;
    private double headingError  = 0;

    private double Lj_init_x = 0.0;
    private double Lj_init_y = 0.0;
    private double Rj_init_x = 0.0;
    private double Rj_init_y = 0.0;
    private double captureBallStartEncoderCount = 0;
    private int delayLoopCount = 0;

    private boolean putting = false;

    enum State {
        OFF,
        FIND_BALL,
        DRIVE_TO_BALL,
        DRIVE_PAST_BALL,
        ROTATE_AROUND,
        CAPTURE_BALL,
        DELAY_LOOP,
        HIT_BALL
    }

    private State currentState = State.OFF;
    private State returnState = State.OFF;

    private void initializeMotors()
    {
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        frontRigtDrive = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "backRightDrive");
        clubMotor = hardwareMap.get(DcMotorEx.class, "clubMotor");

        frontRigtDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRigtDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clubMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRigtDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clubMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clubMotor.setTargetPosition(0);
        clubMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clubMotor.setPower(0.5);
        clubHome();
    }

    private void initializeSensors()
    {
        ballDistance = hardwareMap.get(DistanceSensor.class, "ballDistance");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
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
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
        });

    }

    private void displayTelemetry(){
        /*telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());

        telemetry.addData("Ball X", RobotConstants.ballX);
        telemetry.addData("Ball Y", RobotConstants.ballY);
        telemetry.addData("Ball Found", RobotConstants.ballExists);
        telemetry.addData("Ball area", RobotConstants.foundBallArea);


        telemetry.addData("Club Enc", clubMotor.getCurrentPosition());
*/
        telemetry.addData("range", String.format("%.01f mm", ballDistance.getDistance(DistanceUnit.MM)));
        telemetry.addData("state", currentState);
        telemetry.addData("club current alert setting", clubMotor.getCurrentAlert(CurrentUnit.MILLIAMPS));
        telemetry.addData("club current ", clubMotor.getCurrentAlert(CurrentUnit.MILLIAMPS));

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("1", angles.firstAngle);
        telemetry.addData("2", angles.secondAngle);
        telemetry.addData("3", angles.thirdAngle);

        updateTelemetry(telemetry);
    }

    private void initializeControllers()
    {
        Lj_init_x = gamepad1.left_stick_x;
        Lj_init_y = gamepad1.left_stick_y;

        Rj_init_x = gamepad1.right_stick_x;
        Rj_init_y = gamepad1.right_stick_y;
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    private void findBall() {
        putting = false;

        double rotatePower;
        if (GolfBotIPCVariables.ballExists) {
            if (Math.abs(GolfBotIPCVariables.ballX) < 10) {
                rotatePower = 0.0;
                currentState = State.DRIVE_TO_BALL;
            }
            else if (Math.abs(GolfBotIPCVariables.ballX) < 50)
                rotatePower = GolfBotIPCVariables.ballX * GolfBotMotionConstants.ROTATE_FACTOR_SMALL;
            else if (Math.abs(GolfBotIPCVariables.ballX) < 200)
                rotatePower = GolfBotIPCVariables.ballX * GolfBotMotionConstants.ROTATE_FACTOR_MEDIUM;
            else
                rotatePower = GolfBotIPCVariables.ballX * GolfBotMotionConstants.ROTATE_FACTOR_LARGE;
            // rotatePower = 0.0;
        } else {
            rotatePower = 0.0;
        }

        frontLeftDrive.setPower(-rotatePower);
        frontRigtDrive.setPower(rotatePower);
        backLeftDrive.setPower(-rotatePower);
        backRightDrive.setPower(rotatePower);
    }

    private void driveToBall() {
        if (GolfBotIPCVariables.ballY > GolfBotMotionConstants.readyToGrabY) {
            motorsStop();
            captureBallStartEncoderCount = frontLeftDrive.getCurrentPosition();
            clubBack();
            delayLoopCount = GolfBotMotionConstants.delayTimer;
            returnState = State.CAPTURE_BALL;
            currentState = State.DELAY_LOOP;
        } else {
            motorFwd(GolfBotMotionConstants.captureSpeed, GolfBotIPCVariables.ballX * GolfBotMotionConstants.trackFactor);
        }
    }

    private void drivePastBall() {
        if ((captureBallStartEncoderCount - frontLeftDrive.getCurrentPosition()) < GolfBotMotionConstants.drivePastDistance)
        {
            motorFwd(-GolfBotMotionConstants.drivePastSpeed, 0);
        }
        else
        {
            motorsStop();
            resetHeading();
            currentState = State.ROTATE_AROUND;
        }
    }

    private void rotateAroundBall(){
        double rotatePower = 0.0;

        if (Math.abs(getRawHeading() - headingOffset + 180) > 5) {
            motorFwd(0, GolfBotMotionConstants.rotatePower);
        } else if ((GolfBotIPCVariables.ballExists) && (Math.abs(GolfBotIPCVariables.ballX) < 10)) {
            if (GolfBotIPCVariables.ballExists) {
                if (Math.abs(GolfBotIPCVariables.ballX) < 50) {
                    rotatePower = GolfBotIPCVariables.ballX * GolfBotMotionConstants.ROTATE_FACTOR_SMALL;
                } else if (Math.abs(GolfBotIPCVariables.ballX) < 200) {
                    rotatePower = GolfBotIPCVariables.ballX * GolfBotMotionConstants.ROTATE_FACTOR_MEDIUM;
                } else {
                    rotatePower = GolfBotIPCVariables.ballX * GolfBotMotionConstants.ROTATE_FACTOR_LARGE;
                }

                frontLeftDrive.setPower(-rotatePower);
                frontRigtDrive.setPower(rotatePower);
                backLeftDrive.setPower(-rotatePower);
                backRightDrive.setPower(rotatePower);
            }
        } else {
            motorsStop();
            putting = true;
            currentState = State.DRIVE_TO_BALL;
        }
    }

    private void motorFwd(double speed, double dirError) {
        frontLeftDrive.setPower(speed - dirError);
        frontRigtDrive.setPower(speed + dirError);
        backLeftDrive.setPower(speed - dirError);
        backRightDrive.setPower(speed + dirError);
    }

    private void motorsStop() {
        frontLeftDrive.setPower(0);
        frontRigtDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    private void offState() {
        ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        motorsStop();
    }

    private void captureBall()
    {
        if (ballDistance.getDistance(DistanceUnit.MM) < 160) {
            motorsStop();
            delayLoopCount = GolfBotMotionConstants.delayTimer;
            if (putting) {
                returnState = State.HIT_BALL;

            } else {
                returnState = State.DRIVE_PAST_BALL;
            }

            currentState = State.DELAY_LOOP;
        }
        else {
            motorFwd(GolfBotMotionConstants.captureSpeed, 0);
        }
    }
    private void hitBall()
    {
        clubForward();
        currentState = State.OFF;
    }

    private void delayLoop()
    {
        if (delayLoopCount > 0)
            delayLoopCount --;
        else
            currentState = returnState;
    }

    private void processStateMachine() {
        if (gamepad1.y)
            clubForward();
        if (gamepad1.a)
            clubBack();
        if (gamepad1.b) {
            currentState = State.FIND_BALL;
        }
        if (gamepad1.x) {
            switch (currentState) {
                case OFF:
                    offState();
                    break;
                case FIND_BALL://Rotate the bot to point towards the ball
                    ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
                    findBall();
                    break;
                case DRIVE_TO_BALL://Drive forward, tracking the ball, until the ball is just in front of the bot, then raise the arm
                    ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE);
                    driveToBall();
                    break;
                case DRIVE_PAST_BALL:
                    ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
                    drivePastBall();
                    break;
                case ROTATE_AROUND:
                    ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
                    rotateAroundBall();
                    break;
                case CAPTURE_BALL://Drive forward until the ball is found by the distance sensor
                    ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
                    captureBall();
                    break;
                case HIT_BALL://Swing the club to hit the ball
                    ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
                    hitBall();
                    break;
                case DELAY_LOOP :
                    delayLoop();
                    break;
            }
        } else {
            offState();
        }
    }

    private void clubForward() {
        clubMotor.setPower(GolfBotMotionConstants.hitBallPower);
        clubMotor.setTargetPosition(GolfBotMotionConstants.clubForward);
    }
    private void clubHome() {
        clubMotor.setPower(0.5);
        clubMotor.setTargetPosition(0);
    }

    private void clubBack() {
        clubMotor.setPower(0.3);
        clubMotor.setTargetPosition(GolfBotMotionConstants.clubBack);
    }

    private void initializeMisc()
    {
        ledLights = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
    }

    private void checkErrors()
    {
        //Do some safety checks on the motors etc...
        //Check the motor currents. If any are high then shut everything down and let the user know
        //if (frontLeftDrive.getCurrent())
    }

    public void runOpMode()  {
        initializeDashboard();
        initializeMotors();
        initializeVision();
        initializeSensors();
        initializeMisc();
        initializeControllers();

        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        waitForStart();
        currentState = State.FIND_BALL;
        while (opModeIsActive()) {
            checkErrors();
            displayTelemetry();
            processStateMachine();
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
            Mat DisplayImage = input.clone();
            //Convert to HSV for better color range definition
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
            //Filter pixels outside the desired color range
            Scalar color_lower = new Scalar(GolfBotBallConstants.color_lower_h, GolfBotBallConstants.color_lower_s, GolfBotBallConstants.color_lower_v);
            Scalar color_upper = new Scalar(GolfBotBallConstants.color_upper_h, GolfBotBallConstants.color_upper_s, GolfBotBallConstants.color_upper_v);
            Core.inRange(input, color_lower, color_upper, input);
            //De-speckle the image
            Mat element = Imgproc.getStructuringElement(GolfBotBallConstants.elementType,
                                                        new Size(2 * GolfBotBallConstants.kernelSize + 1,
                                                                 2 * GolfBotBallConstants.kernelSize + 1),
                                                        new Point(GolfBotBallConstants.kernelSize,
                                                                GolfBotBallConstants.kernelSize));
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
                if((a>largest_area) && (a>GolfBotBallConstants.minBallArea)){
                    foundBall = true;
                    largest_area=a;
                    largest_contour_index=i;                //Store the index of largest contour
                    //bounding_rect=boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
                }
            }
            GolfBotIPCVariables.foundBallArea = largest_area;
            //Find the contours and bounding regions
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
            //Not needed now since drawing on the original image Imgproc.cvtColor(input, input, Imgproc.COLOR_GRAY2RGB);
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
                if (GolfBotBallConstants.drawContours == 1) {
                    Imgproc.drawContours(DisplayImage, contoursPolyList, i, color);
                }
                if (GolfBotBallConstants.drawRectangle == 1) {
                    Imgproc.rectangle(DisplayImage, boundRect[i].tl(), boundRect[i].br(), color, 2);
                }
                if (GolfBotBallConstants.drawCircle == 1) {
                    Imgproc.circle(DisplayImage, centers[i], (int) radius[i][0], color, 2);
                }
            }

            GolfBotIPCVariables.ballExists = foundBall;

             if (foundBall) {
                 GolfBotIPCVariables.ballX = centers[largest_contour_index].x - (input.width() / 2.0);
                 GolfBotIPCVariables.ballY = centers[largest_contour_index].y - (input.height() / 2.0);
             }

             /*
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */
            DisplayImage.copyTo(input);
             DisplayImage.release();
            return input;
        }

    }
}

