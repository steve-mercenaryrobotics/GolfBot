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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp (name = "GolfBotSimple", group = "Linear Opmode")

//@Disabled
public class GolfBotSimple extends LinearOpMode {
    //Dashboard variables
    private FtcDashboard dashboard;

    // Vision variables
    OpenCvWebcam webcam;

    // Motor variables
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;
    private DcMotorEx clubMotor = null;
    private DistanceSensor ballDistance = null;
    private RevBlinkinLedDriver ledLights = null;

    // Control/Expansion Hub IMU
    private BNO055IMU imu = null;
    private double robotHeading  = 0;
    private double headingOffset = 0;

    private RevBlinkinLedDriver.BlinkinPattern ledPattern = null;

    private double captureBallStartEncoderCount = 0;
    private int delayLoopCount = 0;

    private boolean putting = false;
    private boolean isRunning = false;

    enum State {
        OFF,
        ERROR,
        FIND_BALL,
        DRIVE_TO_BALL,
        DRIVE_PAST_BALL,
        ROTATE_AROUND,
        CAPTURE_BALL,
        ALIGN_BALL,
        DELAY_LOOP,
        HIT_BALL
    }

    private State currentState = State.OFF;
    private State returnState = State.OFF;

    /**
     * Initialize drive motors, clubMotor; zero club motor encoder
     */
    private void initializeMotors() {
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "backRightDrive");
        clubMotor = hardwareMap.get(DcMotorEx.class, "clubMotor");

        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clubMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clubMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clubMotor.setTargetPosition(0);
        clubMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clubMotor.setPower(0.5);
        clubMotor.setTargetPositionTolerance(GolfBotMotionConstants.clubMotorTolerance);
        clubHome();
    }

    /**
     * Initialize Distance Sensor and BN055IMU
     */
    private void initializeSensors() {
        ballDistance = hardwareMap.get(DistanceSensor.class, "ballDistance");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    /**
     * Initialize FTC Dashboard and Telemetry
     */
    private void initializeDashboard() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    /**
     * Initialize Vision and Pipeline
     */
    private void initializeVision() {
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
                // webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                currentState = State.ERROR;
                telemetry.addLine(String.format(Locale.ENGLISH, "ERROR: Camera Init %d", errorCode));
            }
        });

    }

    /**
     * Initialize Led Lights
     */
    private void initializeMisc() {
        ledLights = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        setLightPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
    }

    /**
     * Display Telemetry Data
     */
    private void displayTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Running", isRunning);

        telemetry.addData("FPS", String.format(Locale.ENGLISH, "%.2f", webcam.getFps()));

        telemetry.addData("Club Enc", clubMotor.getCurrentPosition());
        telemetry.addData("Club Current ", clubMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Distance Range", String.format(Locale.ENGLISH, "%.01f mm", ballDistance.getDistance(DistanceUnit.MM)));

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("First Angle", angles.firstAngle);

        updateTelemetry(telemetry);
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * reset heading angle to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    /**
     * Drive forward at speed and turning speed
     * @param speed Speed to run motors (can be negative)
     * @param dirError Speed to turn (can be negative)
     */
    private void motorFwd(double speed, double dirError) {
        frontLeftDrive.setPower(speed - dirError);
        frontRightDrive.setPower(speed + dirError);
        backLeftDrive.setPower(speed - dirError);
        backRightDrive.setPower(speed + dirError);
    }

    /**
     * Stop all drive motors
     */
    private void motorsStop() {
        // Stop all drive motors
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    /**
     * Move club to forward position
     */
    private void clubForward() {
        clubMotor.setPower(GolfBotMotionConstants.hitBallPower);
        clubMotor.setTargetPosition(GolfBotMotionConstants.clubForward);
    }

    /**
     * Move club to zero position
     */
    private void clubHome() {
        clubMotor.setPower(0.5);
        clubMotor.setTargetPosition(0);
    }

    /**
     * Move club to back position
     */
    private void clubBack() {
        clubMotor.setPower(0.3);
        clubMotor.setTargetPosition(GolfBotMotionConstants.clubBack);
    }

    /**
     * Set pattern of leds
     * @param pattern Blinkin Pattern to use
     */
    private void setLightPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        /*
        Sets Blinkin Pattern
        Don't re-set the pattern if already set to minimize logcat output
         */
        if (pattern != ledPattern) {
            ledLights.setPattern(pattern);
            ledPattern = pattern;
        }
    }

    /**
     * Code that is run when done in state machine loop
     */
    private void offState() {
        if (GolfBotIPCVariables.ballExists) {
            setLightPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
        } else {
            setLightPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY);
        }
        motorsStop();
    }

    /**
     * Error State caused by club over-current
     */
    public void errorState() {
        frontLeftDrive.setMotorDisable();
        frontRightDrive.setMotorDisable();
        backLeftDrive.setMotorDisable();
        backRightDrive.setMotorDisable();
        clubMotor.setMotorDisable();
        setLightPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
    }

    /**
     * Angle towards ball using vision
     */
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
        frontRightDrive.setPower(rotatePower);
        backLeftDrive.setPower(-rotatePower);
        backRightDrive.setPower(rotatePower);
    }

    /**
     * Drive until ball is out of sight
     */
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

    /**
     * Drive past ball using encoders
     */
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

    /**
     * Rotate 180 degrees using imu
     */
    private void rotateAroundBall(){
        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        double headingError = 180 - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        if (Math.abs(headingError) > GolfBotMotionConstants.rotatePrecision) {
            motorFwd(0, GolfBotMotionConstants.rotatePower);
        } else {
            motorsStop();
            putting = true;
            currentState = State.DRIVE_TO_BALL;
        }
    }

    /**
     * Go forward until ball in in line of distance sensor
     */
    private void captureBall() {
        double dist = ballDistance.getDistance(DistanceUnit.MM);
        if (dist < 160) {
            // Ball is captured
            returnState = State.ALIGN_BALL;
            currentState = State.DELAY_LOOP;
        }
        else {
            motorFwd(GolfBotMotionConstants.captureSpeed, 0);
        }
    }

    /**
     * Align ball with distance sensor (unimplemented)
     */
    private void align_ball() {
        // TODO: Implement this
        /*double dist = ballDistance.getDistance(DistanceUnit.MM);
        if ((dist > GolfBotMotionConstants.captureLowerDist) && (dist < GolfBotMotionConstants.captureHigherDist)) {
            motorsStop();
            delayLoopCount = GolfBotMotionConstants.delayTimer;
            if (putting) {
                returnState = State.HIT_BALL;

            } else {
                returnState = State.DRIVE_PAST_BALL;
            }
            currentState = State.DELAY_LOOP;
        } else if (dist > GolfBotMotionConstants.captureHigherDist) {
            frontLeftDrive.setPower(-0.1);
            backRightDrive.setPower(-0.1);
            frontRightDrive.setPower(0.1);
            backLeftDrive.setPower(0.1);
        } else if (dist < GolfBotMotionConstants.captureLowerDist) {
            frontLeftDrive.setPower(0.1);
            backRightDrive.setPower(0.1);
            frontRightDrive.setPower(-0.1);
            backLeftDrive.setPower(-0.1);
        }
        */
        motorsStop();
        delayLoopCount = GolfBotMotionConstants.delayTimer;
        if (putting) {
            returnState = State.HIT_BALL;

        } else {
            returnState = State.DRIVE_PAST_BALL;
        }
        currentState = State.DELAY_LOOP;
    }

    /**
     * Hit ball with clubMotor
     */
    private void hitBall() {
        clubForward();
        currentState = State.OFF;
    }

    /**
     * Delay loop by loop cycles
     */
    private void delayLoop() {
        if (delayLoopCount > 0)
            delayLoopCount --;
        else
            currentState = returnState;
    }

    /**
     * State Machine
     */
    private void processStateMachine() {
        isRunning = gamepad1.x || gamepad1.right_bumper;

        if (gamepad1.y)
            clubForward();
        if (gamepad1.a)
            clubBack();
        if (gamepad1.b || gamepad1.left_bumper) {
            currentState = State.FIND_BALL;
            clubHome();
        }
        if (isRunning) {
            switch (currentState) {
                case OFF:
                    offState();
                    break;
                case ERROR:
                    errorState();
                    break;
                case FIND_BALL://Rotate the bot to point towards the ball
                    setLightPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
                    findBall();
                    break;
                case DRIVE_TO_BALL://Drive forward, tracking the ball, until the ball is just in front of the bot, then raise the arm
                    setLightPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE);
                    driveToBall();
                    break;
                case DRIVE_PAST_BALL:
                    setLightPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE);
                    drivePastBall();
                    break;
                case ROTATE_AROUND:
                    setLightPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE);
                    rotateAroundBall();
                    break;
                case CAPTURE_BALL://Drive forward until the ball is found by the distance sensor
                    setLightPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE);
                    captureBall();
                    break;
                case ALIGN_BALL:
                    align_ball();
                    break;
                case HIT_BALL://Swing the club to hit the ball
                    setLightPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE);
                    hitBall();
                    break;
                case DELAY_LOOP :
                    delayLoop();
                    break;
            }
        } else {
            if (currentState == State.ERROR) {
                errorState();
            } else {
                offState();
            }
        }
    }

    private void checkErrors() {
        // Do some safety checks on the motors etc...
        // Check the motor currents. If any are high then shut everything down and let the user know
        if (clubMotor.getCurrent(CurrentUnit.MILLIAMPS) > GolfBotMotionConstants.maxClubCurrent) {
            currentState = State.ERROR;
        }
    }

    public void runOpMode()  {
        initializeDashboard();
        initializeMotors();
        initializeVision();
        initializeSensors();
        initializeMisc();

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
    static class OurOpenCVPipeline extends OpenCvPipeline {

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        /**
         * Get largest contour index in list of OpenCV contours
         * @param contours contours to search
         * @return contour index
         */
        private int getLargestContourIndex(List<MatOfPoint> contours) {
            // Now search the list for the new largest "blob"
            double largest_area = 0.0;
            int largest_contour_index = -1;

            for (int i = 0; i < contours.size(); i++) {// iterate through each contour.
                double area = Imgproc.contourArea(contours.get(i),false); // Find the area of contour
                if ((area > largest_area) && (area > GolfBotVisionConstants.minBallArea)) {
                    largest_area=area;
                    largest_contour_index=i; // Store the index of largest contour
                }
            }

            return largest_contour_index;
        }

        /**
         * Get largest contour area in list of OpenCV contours
         * @param contours contours to search
         * @return contour area
         */
        private double getLargestContourArea(List<MatOfPoint> contours) {
            // Now search the list for the new largest "blob"
            double largest_area = 0.0;

            for (int i = 0; i < contours.size(); i++) {// iterate through each contour.
                double area = Imgproc.contourArea(contours.get(i),false); // Find the area of contour
                if ((area > largest_area) && (area > GolfBotVisionConstants.minBallArea)) {
                    largest_area=area;
                }
            }

            return largest_area;
        }

        /**
         * Get centers of contour "blobs"
         * @param contours contours to search
         * @return contour centers
         */
        private Point[] getContourCenters(List<MatOfPoint> contours) {
            MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
            Point[] centers = new Point[contours.size()];

            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                centers[i] = new Point();
                Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], null);
            }

            return centers;
        }

        /**
         * Get radii of contour "blobs"
         * @param contours contours to search
         * @return contour radii
         */
        private float[][] getContourRadii(List<MatOfPoint> contours) {
            MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
            float[][] radii = new float[contours.size()][1];

            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                Imgproc.minEnclosingCircle(contoursPoly[i], null, radii[i]);
            }

            return radii;
        }

        /**
         * Get contour polygons
         * @param contours contours to search
         * @return contour polys
         */
        private List<MatOfPoint> getContourPolys(List<MatOfPoint> contours) {
            MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];

            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                Imgproc.minEnclosingCircle(contoursPoly[i], null, null);
            }
            List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
            for (MatOfPoint2f poly : contoursPoly) {
                contoursPolyList.add(new MatOfPoint(poly.toArray()));
            }

            return contoursPolyList;
        }

        @Override
        public Mat processFrame(Mat input) {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */
            Mat DisplayImage = input.clone();

            //Convert to HSV for better color range definition
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

            //Filter pixels outside the desired color range
            Scalar color_lower = new Scalar(GolfBotVisionConstants.color_lower_h, GolfBotVisionConstants.color_lower_s, GolfBotVisionConstants.color_lower_v);
            Scalar color_upper = new Scalar(GolfBotVisionConstants.color_upper_h, GolfBotVisionConstants.color_upper_s, GolfBotVisionConstants.color_upper_v);
            Core.inRange(input, color_lower, color_upper, input);

            //De-speckle the image
            Mat element = Imgproc.getStructuringElement(GolfBotVisionConstants.elementType,
                                                        new Size(2 * GolfBotVisionConstants.kernelSize + 1,
                                                                 2 * GolfBotVisionConstants.kernelSize + 1),
                                                        new Point(GolfBotVisionConstants.kernelSize,
                                                                GolfBotVisionConstants.kernelSize));
            Imgproc.erode(input, input, element);

            // Find blobs in the image. Actually finds a list of contours which will need to be processed later
            List<MatOfPoint> contours = new ArrayList<>();
            final Mat hierarchy = new Mat();
            Imgproc.findContours(input, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Filter out blobs larger than maxBallArea
            contours.removeIf(c -> (Imgproc.contourArea(c) > GolfBotVisionConstants.maxBallArea));

            // Get largest contour (hopefully ball)
            int largest_contour_index = getLargestContourIndex(contours);
            GolfBotIPCVariables.foundBallArea = getLargestContourArea(contours);
            boolean foundBall = largest_contour_index > -1;

            // Find the contours and bounding regions
            Point[] centers = getContourCenters(contours);
            float[][] radii = getContourRadii(contours);
            List<MatOfPoint> contoursPolyList = getContourPolys(contours);

            // Draw shape and/or contour on original image
            Scalar color;
            for (int i = 0; i < contours.size(); i++) {
                if (i == largest_contour_index) {
                    color = new Scalar(0, 255, 0);
                } else {
                    color = new Scalar(255, 0, 0);
                }

                if (GolfBotVisionConstants.drawContours) {
                    Imgproc.drawContours(DisplayImage, contoursPolyList, i, color);
                }
                if (GolfBotVisionConstants.drawCircle) {
                    Imgproc.circle(DisplayImage, centers[i], (int) radii[i][0], color, 2);
                }
            }

            // IPC
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

