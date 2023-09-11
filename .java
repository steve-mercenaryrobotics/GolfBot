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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="HoloTeleOP2", group="Linear Opmode")
//@Autonomous(name="Gyro holo drive", group="Linear opmodes")

//@Disabled
public class Holodrive2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;
    BNO055IMU imu    = null;
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // * CONTROL_HUB_ORIENTATION_FACTOR MUST be set correctly depending on whether control hub is on the top or bottom of the robot!!! * //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private int  CONTROL_HUB_ORIENTATION_FACTOR = -1; // -1 for top, +1 for bottom
    double translateJoyX = 0.0;
    double translateJoyY = 0.0;
    double translateTargetJoyX = 0.0;
    double translateTargetJoyY = 0.0;
    double translateDirection;
    double translateSpeed;
    double targetHeading;
    double currentHeading;
    double botCentricDirection;
    double headingError;
    double normalizationAngle;
    double finalSpeed;
    double normalizationFactor;
    double headingCorrectionSpeed;
    double frontLeftSpeed;
    double frontRightSpeed;
    double backLeftSpeed;
    double backRightSpeed;
    double rotateJoyX;
    boolean rotateClockwiseBumper;
    boolean rotateAnticlockwiseBumper;
    boolean rotateNorth = false;
    boolean rotateSouth = false;
    boolean rotateEast  = false;
    boolean rotateWest  = false;
    public int max_velocity = 2000;
    public int DriveMode = 1;
//    double translateDeadband = 0.05;//Minimum speed factorWill need to tune
//    double rotationSpeedFactor = 1.5;//Sets maximum rotation speed for auto correction. Gets multiplied by error in radians
//    double manualRotationSpeed = .02;//Heading change per loop period in radians. Will need to tune
//    double minimumHeadingCorrectionSpeed = 0.05;//Minimum rotation correction speed. Will need to tune
//    double headingCorrectionDeadband = 0.7;//Deadband in radians. Will need to tune
    double translateDeadband = 0.05;//Minimum speed factor. Will need to tune
    double rotationSpeedFactor = 2;//Sets maximum rotation speed for auto correction. Gets multiplied by error in radians
    double manualRotationSpeed = .2;//Heading change per loop period in radians. Will need to tune
    double minimumHeadingCorrectionSpeed = 0.05;//Minimum rotation correction speed. Will need to tune
    double headingCorrectionDeadband = 0.01;//Deadband in radians. Will need to tune

    /**
     * Initialize the motors
     */
    private void initializeMotors(){
        // Initialize the motor hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftDrive  = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "backRightMotor");

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

        resetEncoders();
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

    private void resetGyro(){
        //  imu.resetZAxisIntegrator();
    }

    private double getGyroHeading(){
        //  return gyro.getIntegratedZValue();
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return -angles.firstAngle * CONTROL_HUB_ORIENTATION_FACTOR;
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

//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();

        // Wait for the game to start (driver presses PLAY)
        //Note, we can use this time to be processing the vision to have the skystone location ready ASAP.
        waitStart();
        runtime.reset();

        //Make sure the gyro is at zero
        resetGyro();

        doTeleop();

        //Done so turn everything off now
        disableHardware();
    }

    private void waitStart(){
        // Wait for the game to start (driver presses PLAY)
        //Can use this time to process the vision in order to get the skystone location ASAP
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
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        initializeMotors();
        initializeGyro();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /**
     * Get the current robot gyro heading in radians
     */
    public double getCurrentHeading(){
        double current;

        current = getGyroHeading();
        //Convert to +- 180 in radians
        while (current > Math.PI)  current -= 2*Math.PI;
        while (current <= -Math.PI) current += 2*Math.PI;
        return current;
    }

    /**
     * Calculate the heading error and limit to +- 180 degrees. Result in radians
     * @param targetHeading, currentHeading
     */

    public double calculateHeadingError(double targetHeading, double currentHeading) {
        double robotError;

        // calculate error in -179 to +180 range : In radians
        robotError = targetHeading - currentHeading;
        while (robotError > Math.PI)  robotError -= 2*Math.PI;
        while (robotError <= -Math.PI) robotError += 2*Math.PI;
        return -robotError;
    }

    private void setMotors(double FL, double FR, double BL, double BR){
        double max;
        //If any value is greater than 1.0 normalize all values accordingly
        max = Math.max(Math.max(FL,FR),Math.max(BL,BR));
        if (max > 1.0){
            FL = FL / max;
            FR = FR / max;
            BL = BL / max;
            BR = BR / max;
        }
        if (DriveMode == 0)
        {
            frontLeftDrive.setPower(FL);
            frontRightDrive.setPower(FR);
            backLeftDrive.setPower(BL);
            backRightDrive.setPower(BR);
        }
        else
        {
            frontLeftDrive.setVelocity(FL * max_velocity);
            frontRightDrive.setVelocity(FR * max_velocity);
            backLeftDrive.setVelocity(BL * max_velocity);
            backRightDrive.setVelocity(BR * max_velocity);
        }
    }

    private void updateMotionJoystick()
    {
        double motionUpdateFactor = 0.02;
        translateTargetJoyX =    -gamepad1.left_stick_x;
        translateTargetJoyY =    -gamepad1.left_stick_y;

        if (translateJoyX + motionUpdateFactor > translateTargetJoyX)
        {
            translateJoyX = translateTargetJoyX;
        }
        else
        {
            translateJoyX = translateJoyX + motionUpdateFactor;
        }
        //Joystick Y tracking
        if (translateTargetJoyY > translateJoyY)
        {
            if (translateJoyY + motionUpdateFactor > translateTargetJoyY)
            {
                translateJoyY = translateTargetJoyY;
            }
            else
            {
                translateJoyY = translateJoyY + motionUpdateFactor;
            }
        }
        else if (translateTargetJoyY < translateJoyY)
        {
            if (translateJoyY - motionUpdateFactor < translateTargetJoyY)
            {
                translateJoyY = translateTargetJoyY;
            }
            else
            {
                translateJoyY = translateJoyY - motionUpdateFactor;
            }
        }
        rotateJoyX =    gamepad1.right_stick_x;
        rotateAnticlockwiseBumper =   gamepad1.left_bumper;
        rotateClockwiseBumper =  gamepad1.right_bumper;
        rotateNorth = gamepad1.dpad_up;
        rotateSouth = gamepad1.dpad_down;
        rotateEast  = gamepad1.dpad_right;
        rotateWest  = gamepad1.dpad_left;
    }

    /**
     * Calculate normalization factor angle (i.e. where in semi-quadrant for unit square to unit circle transposition)
     * @param angle angle to calculate normalization factor for in radians
     */
    private double unitNormalizationAngle(double angle){
        double normalizationAngle;
        double angleDegrees;

        angleDegrees = Math.toDegrees(angle);
        if (angleDegrees >= 0)
            normalizationAngle = angleDegrees % 90;
        else
            normalizationAngle = (-angleDegrees) % 90;

        if (normalizationAngle >= 45) {
            normalizationAngle = 90 - normalizationAngle;
        }

        //
        return Math.toRadians(normalizationAngle);
    }

    void updateTargetSpeedDirection()
    {
        //Calculate desired translation 'speed' and  direction
        translateSpeed = Math.sqrt((translateJoyX * translateJoyX) + (translateJoyY * translateJoyY));

        if (translateSpeed > translateDeadband){//Create deadband and also ensure no divide by zero in atan2 and stop robot twitching
            //Calculate the desired robot base direction
            //Forward = 0 radians (0 degrees)
            translateDirection = (-Math.atan2(translateJoyY, translateJoyX) + (Math.PI/2));
        }
        else {
            translateDirection = 0;
            translateSpeed = 0;
        }
    }

    void updateTargetHeading()
    {
        if (rotateNorth)
        {
            targetHeading = Math.toRadians(0);
        }
        else if (rotateSouth)
        {
            targetHeading = Math.toRadians(180);
        }
        else if (rotateWest)
        {
            targetHeading = Math.toRadians(90);
        }
        else if (rotateEast)
        {
            targetHeading = Math.toRadians(270);
        }

        //Now check if any rotation requested from the bumpers
        if (rotateAnticlockwiseBumper) {
            targetHeading = currentHeading + manualRotationSpeed;
        }
        else if (rotateClockwiseBumper) {
            targetHeading = currentHeading - manualRotationSpeed;
        }
        //Otherwise check if any rotation requested from the joystick
        else if (Math.abs(rotateJoyX) > 0.1) {
            targetHeading = currentHeading - (rotateJoyX * manualRotationSpeed);
        }
    }

    void updateMotionCorrections()
    {
        headingError = calculateHeadingError(targetHeading, currentHeading);
        //If the error is small (less than the deadband) then do not correct anything
        if (Math.abs(headingError) > headingCorrectionDeadband)
            headingCorrectionSpeed = headingError * rotationSpeedFactor;
        else
            headingCorrectionSpeed = 0.0;

        //If the correction power is really small then increase it so it actually does something, unless it really was zero
        if ((Math.abs(headingCorrectionSpeed) < minimumHeadingCorrectionSpeed)  && (headingCorrectionSpeed != 0.0))
        {
            if (headingCorrectionSpeed >= 0)
                headingCorrectionSpeed = minimumHeadingCorrectionSpeed;
            else
                headingCorrectionSpeed = -minimumHeadingCorrectionSpeed;
        }

        //Subtract the current heading to get robot centric direction
        botCentricDirection =  translateDirection - currentHeading;

        //This version will normalize the joystick position to fully utilize the entire power range for 0, 90, 180 & 270 by mapping from unit square to unit circle
        //Adjust to use full range of speed to 'boost' the power for 0, 90, 180, 270 from .707 to 1.0
        normalizationAngle = unitNormalizationAngle(targetHeading);
        //The following can be optimized to eliminate some sqrt calls
        normalizationFactor = Math.sqrt(1+Math.tan(normalizationAngle))/Math.sqrt(2);
        finalSpeed = translateSpeed / (normalizationFactor * Math.sqrt(2));
        //    //This version is a simplified version that does not maximize the entire power range of the motors
        //    finalSpeed = translateSpeed;
        //    normalizationFactor = 1.0;
    }

    void updateMotorSpeeds()
    {
        //Note, the diagonally opposite speeds are the same, so only need to calculate 2 values
        frontLeftSpeed = finalSpeed*Math.sin(botCentricDirection+(Math.PI/4));
        //Now scale to utilize full power
        frontLeftSpeed = (frontLeftSpeed/normalizationFactor);

        frontRightSpeed = finalSpeed*Math.cos(botCentricDirection+(Math.PI/4));
        //Now scale to utilize full power
        frontRightSpeed = (frontRightSpeed/normalizationFactor);

        //Duplicate diagonal speeds for translation and add in rotation
        backLeftSpeed   = frontRightSpeed + headingCorrectionSpeed;
        backRightSpeed  = frontLeftSpeed - headingCorrectionSpeed;
        frontRightSpeed = frontRightSpeed - headingCorrectionSpeed;
        frontLeftSpeed  = frontLeftSpeed + headingCorrectionSpeed;
    }

    /**
     * Tele-op example
     * Right stick will move the robot in field centric world view
     * Left stick will point the robot in the field centric direction
     * Left/right bumbers will rotate the robot
     */
    private void doTeleop()
    {
        //Take our initial zero heading from our starting pose
        targetHeading = getCurrentHeading();

        while(opModeIsActive())
        {
            currentHeading = getCurrentHeading();
            //Get the joystick states
            updateMotionJoystick();
            //Calculate translation speed and direction from joysticks
            updateTargetSpeedDirection();
            //Update rotation heading
            updateTargetHeading();
            //Calculate if any rotation is needed to point bot in 'targetHeading' direction
            updateMotionCorrections();
            //Now calculate the actual motor speeds
            updateMotorSpeeds();
            //And actually set the motors accordingly
            //Note, this function will also clamp and scale the power to 1.0
            setMotors(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);

            telemetry.addData("FL: ", frontLeftSpeed);
            telemetry.addData("FR: ", frontRightSpeed);
            telemetry.addData("BL: ", backLeftSpeed);
            telemetry.addData("BR: ", backRightSpeed);
            telemetry.update();
        }
        setMotors(0, 0, 0, 0);
    }
}

