/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@TeleOp(name="SteveTestTele", group="Linear Opmode")

@Disabled
public class SteveTestTele extends LinearOpMode {

    final private boolean dashboardEnabled = false;//Are we debugging with FtcDashboard ?
    final private int gyroOrientation = -1;//Set to either +1 or -1 depending on control hub mountint orientation
    final private boolean fieldCentric = true;//Select field centric or robot centric
    //Tunable variables
    final private double translationUpdateFactor = 0.05;//Translation joystick tracking rate
    final private double headingCorrectionFactor = 360.0 / 15000.0;//Heading rotation correction 'power' factor
    final private double minHeadingError = 4.0;//Minimim rotation heading error to actually act on
    final private double rotationCrossover = 15;//Transition point from max to proportional rotation
    final private double rotationCorrectionSpeedMax = 0.5;//The maximum rotation power factor
    final private int    max_base_velocity = 2000;//Max robot base translation velocity
    final private int    max_arm_velocity= 2000;
    final private int    DriveMode = 1;
    final private double translateDeadband = 0.05;//Minimum speed factor. Will need to tune
    final private double rotateTriggerDeadband = 0.1;
    final private double rotationTriggerSpeed = 15.0;//Controls how fast manual triggers cause rotation
    final private double slowSpeedFactor = 0.5;
    final private double fastSpeedFactor = 1.0;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;
    private BNO055IMU imu    = null;
    private double translateJoyX = 0.0;
    private double translateJoyY = 0.0;
    private double translateTargetJoyX = 0.0;
    private double translateTargetJoyY = 0.0;
    private double translationHeading = 0.0;
    private double targetRotation = 0.0;
    private double currentRotation = 0.0;
    private double botCentricTranslationDirection = 0.0;
    private double rotationError = 0.0;
    private double translationSpeed = 0.0;
    private double rotationCorrectionSpeed = 0.0;
    private double frontLeftSpeed;
    private double frontRightSpeed;
    private double backLeftSpeed;
    private double backRightSpeed;
    private double rotateClockwiseTrigger;
    private double rotateAnticlockwiseTrigger;
    private boolean rotateNorth = false;
    private boolean rotateSouth = false;
    private boolean rotateEast  = false;
    private boolean rotateWest  = false;
    private boolean driveSlow = true;//Slow the robot unless bumper pressed
    private double Lj1_init_x;
    private double Lj1_init_y;
    private double Rj1_init_x;
    private double Rj1_init_y;
    private double Lj2_init_x;
    private double Lj2_init_y;
    private double Rj2_init_x;
    private double Rj2_init_y;

    /**
     * Initialize the motors
     */
    private void initializeMotors(){
        // Initialize the motor hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        backLeftDrive  = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "backRightDrive");

        // Motors on one side need to effectively run 'backwards' to move 'forward'
        // Reverse the motors that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorEx.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();
    }

    public void initializeJoysticks(){
        //Capture the 'at rest' positions of the joysticks
        Lj1_init_x = gamepad1.left_stick_x;
        Lj1_init_y = gamepad1.left_stick_y;
        Rj1_init_x = gamepad1.right_stick_x;
        Rj1_init_y = gamepad1.right_stick_y;

        Lj2_init_x = gamepad2.left_stick_x;
        Lj2_init_y = gamepad2.left_stick_y;
        Rj2_init_x = gamepad2.right_stick_x;
        Rj2_init_y = gamepad2.right_stick_y;
    }

    private void resetEncoders(){
        //Stop the motors and reset the encoders to zero
        frontLeftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //Make sure we re-enable the use of encoders
        frontLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Initialize the gyro
     */
    private void initializeGyro(){
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
    }

    private double getGyroHeading(){
        //  return gyro.getIntegratedZValue();
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    private void waitStart(){
        // Wait for the game to start (driver presses PLAY)
        //Can use this time to process the vision in order to get the vision marker ASAP
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            idle();
        }
    }

    /**
     * Disable all hardware
     */
    private void disableHardware() {
        //Only the motors really need to be turned off at the moment
        setMotors(0, 0, 0, 0);
    }

    /**
     * Initialize all hardware
     */
    private void initializeHardware(){
        telemetry.addData("Status", "Initializing hardware");
        telemetry.update();

        initializeMotors();
        initializeGyro();
        initializeJoysticks();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized hardware");
        telemetry.update();
    }

    /**
     * Get the current robot gyro heading in degrees
     */
    public double getCurrentHeading(){
        double current;

        current = getGyroHeading() * gyroOrientation;
        //Convert to +- 180 in radians
        while (current > 180)  current -= 360;
        while (current <= -180) current += 360;
        return current;
    }

    /**
     *
     * @param value Variable to center
     * @param range Positive/negative loop range to center on
     * @return
     */
    private double centerRange(double value, double range)
    {
        double bounds = range * 2;
        while (value >= range)  value -= bounds;
        while (value <= -range) value += bounds;
        return value;
    }
    /**
     * Calculate the heading error and limit to +- 180 degrees. Result in degrees
     * @param target, current
     */
    public double calculateHeadingError(double target, double current) {
        double robotError;

        // calculate error in -179 to +180 range : In degrees
        robotError = centerRange(target - current, 180);

        //If error is small then ignore it
        if   ((robotError > -minHeadingError) && (robotError < minHeadingError))
        {
            robotError = 0.0;
        }
        return robotError;
    }

    private void setMotors(double FL, double FR, double BL, double BR){
        double max;
        //If any value is greater than 1.0 normalize all values accordingly
        max = Math.max(Math.max(Math.max(FL,FR),Math.max(BL,BR)), 1.0);
        FL = FL / max;
        FR = FR / max;
        BL = BL / max;
        BR = BR / max;
        //Depending on whether using power mode or velocity mode...
        //Negate directions here so can maintain consistent configurations in initialization
        if (DriveMode == 0)
        {
            frontLeftDrive.setPower(-FL);
            frontRightDrive.setPower(-FR);
            backLeftDrive.setPower(-BL);
            backRightDrive.setPower(-BR);
        }
        else
        {
            frontLeftDrive.setVelocity(-FL * max_base_velocity);
            frontRightDrive.setVelocity(-FR * max_base_velocity);
            backLeftDrive.setVelocity(-BL * max_base_velocity);
            backRightDrive.setVelocity(-BR * max_base_velocity);
        }
    }            //Negate directions here so can maintain consistent configurations in initialization

    /**
     * Measure the joystick values, apply the initial offset corrections
     * and then implement some smoothing to limit jerky transitions
     */
    private void updateJoysticks()
    {
        //Bot motion controls on stick 1
        translateTargetJoyX        = gamepad1.left_stick_x  - Lj1_init_x;
        translateTargetJoyY        = gamepad1.left_stick_y  - Lj1_init_y;
        driveSlow                  = !gamepad1.right_bumper;
        rotateClockwiseTrigger     = gamepad1.right_trigger;
        rotateAnticlockwiseTrigger = gamepad1.left_trigger;
        rotateNorth                = gamepad1.dpad_up;
        rotateSouth                = gamepad1.dpad_down;
        rotateEast                 = gamepad1.dpad_right;
        rotateWest                 = gamepad1.dpad_left;

        //For the moment don't do any acceleration control
        //translateJoyX = translateTargetJoyX;
        //translateJoyY = translateTargetJoyY;


        if (translateJoyX + translationUpdateFactor > translateTargetJoyX)
        {
            translateJoyX = translateTargetJoyX;
        }
        else
        {
            translateJoyX = translateJoyX + translationUpdateFactor;
        }
        //Joystick Y tracking
        if (translateTargetJoyY > translateJoyY)
        {
            if (translateJoyY + translationUpdateFactor > translateTargetJoyY)
            {
                translateJoyY = translateTargetJoyY;
            }
            else
            {
                translateJoyY = translateJoyY + translationUpdateFactor;
            }
        }
        else if (translateTargetJoyY < translateJoyY)
        {
            if (translateJoyY - translationUpdateFactor < translateTargetJoyY)
            {
                translateJoyY = translateTargetJoyY;
            }
            else
            {
                translateJoyY = translateJoyY - translationUpdateFactor;
            }
        }

        if (rotateAnticlockwiseTrigger < rotateTriggerDeadband)
        {
            rotateAnticlockwiseTrigger = 0;
        }
        if (rotateClockwiseTrigger < rotateTriggerDeadband)
        {
            rotateClockwiseTrigger = 0;
        }
    }

    void updateTargetTranslation()
    {
        //Calculate desired translation 'speed' and  direction
        translationSpeed = Math.sqrt((translateJoyX * translateJoyX) + (translateJoyY * translateJoyY));

        if (translationSpeed > translateDeadband){//Create deadband and also ensure no divide by zero in atan2 and stop robot twitching
            //Calculate the desired robot base direction
            //Forward = 0 radians (0 degrees)
            translationHeading = centerRange(Math.toDegrees(-Math.atan2(translateJoyY, translateJoyX)) - 90, 180);
        }
        else {
            translationHeading = 0;
            translationSpeed = 0;
        }
    }

    void updateTargetRotation()
    {
        if (rotateAnticlockwiseTrigger != 0)
        {
            targetRotation = currentRotation - (rotateAnticlockwiseTrigger) * rotationTriggerSpeed;
        }
        else if (rotateClockwiseTrigger != 0)
        {
            targetRotation = currentRotation + (rotateClockwiseTrigger) * rotationTriggerSpeed;
        }
        else if (rotateNorth)
        {
            targetRotation = 0;
        }
        else if (rotateSouth)
        {
            targetRotation = 180;
        }
        else if (rotateWest)
        {
            targetRotation = 90;
        }
        else if (rotateEast)
        {
            targetRotation = -90;
        }
    }

    void calculateMotorSpeeds()
    {
        double speedAdjust;

        if (driveSlow)
        {
            speedAdjust = slowSpeedFactor;
        }
        else
        {
            speedAdjust = fastSpeedFactor;
        }
        //Note, the diagonally opposite speeds are the same, so only need to calculate 2 values
        frontLeftSpeed = speedAdjust * translationSpeed*Math.sin(Math.toRadians(botCentricTranslationDirection) + (Math.PI/4));
        frontRightSpeed = speedAdjust * translationSpeed*Math.cos(Math.toRadians(botCentricTranslationDirection) + (Math.PI/4));

        //If the error angle is small then use proportional, otherwise use max power
        if (Math.abs(rotationError) < rotationCrossover)
        {
            rotationCorrectionSpeed = rotationError * headingCorrectionFactor;
        }
        else
        {
            if (rotationError >0) {
                rotationCorrectionSpeed = rotationCorrectionSpeedMax;
            }
            else {
                rotationCorrectionSpeed = -rotationCorrectionSpeedMax;
            }
        }
        //Clamp the rotational component to something like 0.5 to allow for both rotation and translation
        if (rotationCorrectionSpeed > rotationCorrectionSpeedMax)
        {
            rotationCorrectionSpeed = rotationCorrectionSpeedMax;
        }
        else if (rotationCorrectionSpeed < -rotationCorrectionSpeedMax)
        {
            rotationCorrectionSpeed = -rotationCorrectionSpeedMax;
        }

        //Duplicate diagonal speeds for translation and add in rotation
        backLeftSpeed   = frontRightSpeed + rotationCorrectionSpeed;
        backRightSpeed  = frontLeftSpeed - rotationCorrectionSpeed;
        frontRightSpeed = frontRightSpeed - rotationCorrectionSpeed;
        frontLeftSpeed  = frontLeftSpeed + rotationCorrectionSpeed;
    }

    void processTelemetry()
    {
        telemetry.addData("FL: ", frontLeftSpeed);
        telemetry.addData("FR: ", frontRightSpeed);
        telemetry.addData("BL: ", backLeftSpeed);
        telemetry.addData("BR: ", backRightSpeed);
        telemetry.addData("Joystick heading",translationHeading);
        telemetry.addData("Target rotation",targetRotation);
        telemetry.addData("Robot rotation",currentRotation);
        telemetry.addData("Rotation error",rotationError);
        telemetry.addData("Robot heading",botCentricTranslationDirection);
        telemetry.addData("rotationCorrectionSpeed",rotationCorrectionSpeed);

        telemetry.update();
    }

    private void doTele()
    {
        //Take our initial zero heading from our starting pose
        targetRotation = getCurrentHeading();

        while(opModeIsActive())
        {
            currentRotation = getCurrentHeading();
            //Get the joystick states and process and softening controls
            updateJoysticks();
            //Calculate translation speed and direction from joysticks
            updateTargetTranslation();
            //Update rotation heading from joysticks
            updateTargetRotation();
            //Calculate the robot's rotation error
            rotationError = calculateHeadingError(targetRotation, currentRotation);
            //Subtract the rotation error so that the translation direction follows the
            //robot's actual rotation and not the ideal rotation
            if (fieldCentric)
            {
                botCentricTranslationDirection = translationHeading + currentRotation + rotationError;
            }
            else {
                botCentricTranslationDirection = translationHeading + rotationError;
            }
            //Now calculate the actual motor speeds
            calculateMotorSpeeds();
            //And now set the motors accordingly
            //Note, this function will also clamp and scale the power to 1.0
            setMotors(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);

            processTelemetry();
        }
        setMotors(0, 0, 0, 0);
    }

    /**
     * This is the main op mode and should call all the initialization, wait for start,
     * execute your desired auto/tele-op, then stop everything
     */
    @Override
    public void runOpMode() {

        //Must be called at the start to initialize all necessary hardware
        //Add other hardware (e.g. vision etc...) in this method
        initializeHardware();

        if (dashboardEnabled) {
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();
        }
        // Wait for the game to start (driver presses PLAY)
        //Note, we can use this time to be processing the vision to have the icon image ready ASAP.
        waitStart();
        runtime.reset();
        //Done so turn everything off now
        disableHardware();
    }

}
