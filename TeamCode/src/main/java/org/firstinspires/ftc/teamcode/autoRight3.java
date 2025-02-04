package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="autoRight3")
public class autoRight3 extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBR;
    private DcMotor motorBL;
    private DcMotor motorArm;
    private DcMotor motorSlide;
    private Servo servoSampleClaw;
    private Servo servoSpecimenClaw;
    private IMU imu;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_INCH = 44.62;
    static final double DRIVE_SPEED = 0.6;
    static  final double ARM_SPEED = 0.4;
    static final double SLIDE_SPEED = 0.5;
    static final double SLOW_SPEED = 0.4;
    static final int YAW_PRECISION = 500;

    private int idealPosMotorFL = 0;
    private int idealPosMotorFR = 0;
    private int idealPosMotorBL = 0;
    private int idealPosMotorBR = 0;
    YawPitchRollAngles robotOrientation;
    double yaw;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        motorSlide = hardwareMap.get(DcMotor.class, "motorSlide");
        servoSampleClaw = hardwareMap.get(Servo.class, "servoSampleClaw");
        servoSpecimenClaw = hardwareMap.get(Servo.class, "servoSpecimenClaw");
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorArm.setDirection(DcMotor.Direction.FORWARD);
        motorSlide.setDirection(DcMotor.Direction.FORWARD);
        servoSampleClaw.setDirection(Servo.Direction.FORWARD);
        servoSpecimenClaw.setDirection(Servo.Direction.FORWARD);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        imu.resetYaw();

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses START)
        telemetry.addData("stuff", 0);
        telemetry.update();
        waitForStart();

        servoSampleClaw.setPosition(0.025);
        servoSpecimenClaw.setPosition(0.5);
        tankDrive(SLOW_SPEED,  23,  23, 0, YAW_PRECISION, 2);
        setArmPos(ARM_SPEED, 400, 2, true);
        setSlidePos(SLIDE_SPEED, 2200, 3);
        setArmPos(ARM_SPEED, 450, 1, true);
        setSlidePos(SLIDE_SPEED, 0, 2);
        tankDrive(SLOW_SPEED, -5, -5, 0, YAW_PRECISION, 1);
        setArmPos(ARM_SPEED, 50, 2, false);
        sideDrive(DRIVE_SPEED, 32, 2);
        tankDrive(DRIVE_SPEED, 35, 35, 0, YAW_PRECISION, 2);
        sideDrive(DRIVE_SPEED, 11, 1);
        tankDrive(DRIVE_SPEED, -47, -47, 0, YAW_PRECISION, 3);
        tankDrive(DRIVE_SPEED, 17, 17, 0, YAW_PRECISION, 2);
        tankDrive(DRIVE_SPEED, -21, -21, 0, YAW_PRECISION, 2);
        tankDrive(SLOW_SPEED, -5, -5, 0, YAW_PRECISION, 1);
        tankDrive(DRIVE_SPEED, 10, 10, 0, YAW_PRECISION, 2);
        servoSpecimenClaw.setPosition(0.7);
        sleep(100);
        setArmPos(ARM_SPEED, 980, 3, false);
        setSlidePos(SLIDE_SPEED, 0, 1);
        tankDrive(0.2, -2, -2, 0, YAW_PRECISION, 2);
        servoSpecimenClaw.setPosition(0.5);
        sleep(500);
        setSlidePos(SLIDE_SPEED, 400, 1);
        tankDrive(SLOW_SPEED, 10, 10, 0, YAW_PRECISION, 2);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void tankDrive(double speed, double leftInches, double rightInches, double angle, int precision, double timeoutS) {
        int newFLtarget;
        int newBLtarget;
        int newFRtarget;
        int newBRtarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFLtarget = idealPosMotorFL += (int)(leftInches * COUNTS_PER_INCH);
            newBLtarget = idealPosMotorBL += (int)(leftInches * COUNTS_PER_INCH);
            newFRtarget = idealPosMotorFR += (int)(rightInches * COUNTS_PER_INCH);
            newBRtarget = idealPosMotorBR += (int)(rightInches * COUNTS_PER_INCH);
            motorFL.setTargetPosition(newFLtarget);
            motorBL.setTargetPosition(newBLtarget);
            motorFR.setTargetPosition(newFRtarget);
            motorBR.setTargetPosition(newBRtarget);

            // Turn On RUN_TO_POSITION
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorBL.setPower(Math.abs(speed));
            motorFL.setPower(Math.abs(speed));
            motorFR.setPower(Math.abs(speed));
            motorBR.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (motorFL.isBusy() && motorBL.isBusy() && motorFR.isBusy() && motorBR.isBusy())) {
                if (!Double.isNaN(angle)) {
                    robotOrientation = imu.getRobotYawPitchRollAngles();
                    yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                    double yawError = Math.abs(((540 + yaw - angle) % 360) - 180);

                    motorBL.setPower(clamp(Math.abs(speed) + yawError / precision, 0, 1));
                    motorFL.setPower(clamp(Math.abs(speed) - yawError / precision, 0, 1));
                    motorFR.setPower(clamp(Math.abs(speed) + yawError / precision, 0, 1));
                    motorBR.setPower(clamp(Math.abs(speed) - yawError / precision, 0, 1));

                    telemetry.addData("yawError", yawError);
                    telemetry.update();
                }
            }

            // Stop all motion;
            motorBL.setPower(0);
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBR.setPower(0);

            sleep(50);


            motorBL.setPower(0.2);
            motorFL.setPower(0.2);
            motorFR.setPower(0.2);
            motorBR.setPower(0.2);


            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (motorFL.isBusy() && motorBL.isBusy() && motorFR.isBusy() && motorBR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newFLtarget, newBLtarget, newFRtarget, newBRtarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d", motorFL.getCurrentPosition(), motorBL.getCurrentPosition(), motorFR.getCurrentPosition(), motorBR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorBL.setPower(0);
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBR.setPower(0);

            // Turn off RUN_TO_POSITION
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(50);   // optional pause after each move.
        }
    }
    public void sideDrive(double speed, double sideDistance, double timeoutS) {
        int newFLtarget;
        int newBLtarget;
        int newFRtarget;
        int newBRtarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFLtarget = idealPosMotorFL += (int)(sideDistance * COUNTS_PER_INCH);
            newBLtarget = idealPosMotorBL -= (int)(sideDistance * COUNTS_PER_INCH);
            newFRtarget = idealPosMotorFR -= (int)(sideDistance * COUNTS_PER_INCH);
            newBRtarget = idealPosMotorBR += (int)(sideDistance * COUNTS_PER_INCH);
            motorFL.setTargetPosition(newFLtarget);
            motorBL.setTargetPosition(newBLtarget);
            motorFR.setTargetPosition(newFRtarget);
            motorBR.setTargetPosition(newBRtarget);

            // Turn On RUN_TO_POSITION
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorBL.setPower(Math.abs(speed));
            motorFL.setPower(Math.abs(speed));
            motorFR.setPower(Math.abs(speed));
            motorBR.setPower(Math.abs(speed));


            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (motorFL.isBusy() && motorBL.isBusy() && motorFR.isBusy() && motorBR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newFLtarget, newBLtarget, newFRtarget, newBRtarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d", motorFL.getCurrentPosition(), motorBL.getCurrentPosition(), motorFR.getCurrentPosition(), motorBR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorBL.setPower(0);
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBR.setPower(0);

            // Turn off RUN_TO_POSITION
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);   // optional pause after each move.
        }
    }
    public void setArmPos(double speed, int armPos, double timeoutS, boolean holdPos) {

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            motorArm.setTargetPosition(armPos);

            // Turn On RUN_TO_POSITION
            motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorArm.setPower(Math.abs(speed));


            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (motorArm.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d", armPos);
                telemetry.addData("Currently at",  " at %7d", motorArm.getCurrentPosition());
                telemetry.update();
            }

            // Turn off RUN_TO_POSITION and stop motion
            if (!holdPos) {
                motorArm.setPower(0);
                motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            sleep(100);   // optional pause after each move.
        }
    }
    public void setSlidePos(double speed, int slidePos, double timeoutS) {

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            motorSlide.setTargetPosition(slidePos);

            // Turn On RUN_TO_POSITION
            motorSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorSlide.setPower(Math.abs(speed));


            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (motorSlide.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d", slidePos);
                telemetry.addData("Currently at",  " at %7d", motorSlide.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorSlide.setPower(0);

            // Turn off RUN_TO_POSITION
            motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);   // optional pause after each move.
        }
    }
    public void turnToAngle(double speed, double angle, double timeoutS) {
        int startPositionBL = motorBL.getCurrentPosition();
        int startPositionFL = motorFL.getCurrentPosition();
        int startPositionFR = motorFR.getCurrentPosition();
        int startPositionBR = motorBR.getCurrentPosition();
        runtime.reset();

        robotOrientation = imu.getRobotYawPitchRollAngles();
        yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        double yawDifference = Math.abs(((540 + yaw - angle) % 360) - 180);

        while (opModeIsActive() && (runtime.seconds() < timeoutS) && (Math.abs(yawDifference) > 10)) {
            robotOrientation = imu.getRobotYawPitchRollAngles();
            yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
            yawDifference = Math.abs(((540 + yaw - angle) % 360) - 180);

            motorBL.setPower((yawDifference / Math.abs(yawDifference)) * -Math.abs(speed));
            motorFL.setPower((yawDifference / Math.abs(yawDifference)) * -Math.abs(speed));
            motorFR.setPower((yawDifference / Math.abs(yawDifference)) * Math.abs(speed));
            motorBR.setPower((yawDifference / Math.abs(yawDifference)) * Math.abs(speed));

            telemetry.addData("yaw", yaw);
            telemetry.addData("yawDifference", yawDifference);
            telemetry.update();
        }

        motorBL.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);

        sleep(100);

        idealPosMotorBL += motorBL.getCurrentPosition() - startPositionBL;
        idealPosMotorFL += motorFL.getCurrentPosition() - startPositionFL;
        idealPosMotorFR += motorFR.getCurrentPosition() - startPositionFR;
        idealPosMotorBR += motorBR.getCurrentPosition() - startPositionBR;
    }
    public double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }
}