package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp (name = "teleopMove")
public class teleopMove extends LinearOpMode{
    public DcMotor motorFL;
    public DcMotor motorFR;
    public DcMotor motorBR;
    public DcMotor motorBL;
    public DcMotor motorArm;
    public DcMotor motorSlide;
    public Servo servoSampleClaw;
    public Servo servoSpecimenClaw;
    private IMU imu;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_INCH = 44.62;
    static final double DRIVE_SPEED = 0.6;
    static  final double ARM_SPEED = 0.6;
    static final double SLIDE_SPEED = 1;
    static final double SLOW_SPEED = 0.4;
    static final int YAW_PRECISION = 500;

    private int idealPosMotorFL = 0;
    private int idealPosMotorFR = 0;
    private int idealPosMotorBL = 0;
    private int idealPosMotorBR = 0;
    YawPitchRollAngles robotOrientation;
    double yaw;
    public double speedMode = 1;
    public double servoSamplePos = 0.025;
    public double servoSpecimenPos = 0.5;
    public int armPos = 0;
    boolean gamepad2ButtonA = false;
    boolean gamepad2ButtonY = false;
    boolean gamepad2ButtonB = false;
    boolean gamepad1ButtonA = false;
    boolean controlledServo = false;

    boolean gamepad2ButtonYTriggered = false;

    public void runOpMode() throws InterruptedException {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBR = hardwareMap.get(DcMotor.class, "motorBL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBR");
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        motorSlide = hardwareMap.get(DcMotor.class, "motorSlide");
        servoSampleClaw = hardwareMap.get(Servo.class, "servoSampleClaw");
        servoSpecimenClaw = hardwareMap.get(Servo.class, "servoSpecimenClaw");

        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorArm.setDirection(DcMotor.Direction.FORWARD);
        motorSlide.setDirection(DcMotor.Direction.FORWARD);
        servoSampleClaw.setDirection(Servo.Direction.FORWARD);
        servoSpecimenClaw.setDirection(Servo.Direction.FORWARD);

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorArm.setPower(0);
        motorSlide.setPower(0);

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

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        servoSampleClaw.setPosition(servoSamplePos);
        servoSpecimenClaw.setPosition(servoSpecimenPos);

        while (opModeIsActive()) {

            armPos += (int) (5 * (gamepad2.right_trigger - gamepad2.left_trigger));
            double slidePower = ((gamepad2.right_bumper ? 1 : 0) - (gamepad2.left_bumper ? 1 : 0));
            if (motorSlide.getCurrentPosition() < 20 && !(gamepad2.left_stick_button && gamepad2.left_stick_y > 0.6)) {
                slidePower = Math.max(slidePower, 0);
            }
            if (motorSlide.getCurrentPosition() > 2000 && !(motorArm.getCurrentPosition() > 620) && !(gamepad2.left_stick_button && gamepad2.left_stick_y > 0.6)) {
                slidePower = Math.min(slidePower, 0);
            }
            if (motorSlide.getCurrentPosition() > 3000 && !(gamepad2.left_stick_button && gamepad2.left_stick_y > 0.6)) {
                slidePower = Math.min(slidePower, 0);
            }
            if (gamepad2.left_stick_button && gamepad2.left_stick_y < 0.6) {
                motorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (gamepad2.right_stick_button && gamepad2.right_stick_y < 0.6) {
                motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            double forward = speedMode * Math.pow(gamepad1.left_stick_y, 3);
            double right = -speedMode * Math.pow(gamepad1.right_stick_x, 3);
            double turn = -speedMode * Math.pow(gamepad1.left_stick_x, 3);
            double leftFrontPower = forward + right + turn;
            double leftBackPower = forward - right + turn;
            double rightFrontPower = forward - right - turn;
            double rightBackPower = forward + right - turn;
            double[] powers = {leftFrontPower, leftBackPower, rightFrontPower, rightBackPower};
            boolean needToScale = false;
            boolean prevGamepad2ButtonA = gamepad2ButtonA;
            boolean prevGamepad2ButtonB = gamepad2ButtonB;
            boolean prevGamepad1ButtonA = gamepad1ButtonA;
            gamepad2ButtonA = gamepad2.a;
            gamepad2ButtonB = gamepad2.b;
            gamepad1ButtonA = gamepad1.a;
            for (double power : powers) {
                if (Math.abs(power) > 1) {
                    needToScale = true;
                    break;
                }
            }
            if (needToScale) {
                double greatest = 0;
                for (double power : powers) {
                    if (Math.abs(power) > greatest) {
                        greatest = Math.abs(power);
                    }
                }
                leftFrontPower /= greatest;
                leftBackPower /= greatest;
                rightFrontPower /= greatest;
                rightBackPower /= greatest;
            }
            boolean stop = true;
            for (double power : powers) {
                double stopBuffer = 0;
                if (Math.abs(power) > stopBuffer) {
                    stop = false;
                    break;
                }
            }
            if (gamepad1.right_trigger > 0.2 || gamepad1.left_trigger > 0.2) {
                leftFrontPower = leftFrontPower/(3 + ((gamepad1.right_trigger > 0.2 && gamepad1.left_trigger > 0.2) ? 3 : 0));
                leftBackPower = leftBackPower/(3 + ((gamepad1.right_trigger > 0.2 && gamepad1.left_trigger > 0.2) ? 3 : 0));
                rightFrontPower = rightFrontPower/(3 + ((gamepad1.right_trigger > 0.2 && gamepad1.left_trigger > 0.2) ? 3 : 0));
                rightBackPower = rightBackPower/(3 + ((gamepad1.right_trigger > 0.2 && gamepad1.left_trigger > 0.2) ? 3 : 0));
            }
            if (stop) {
                leftFrontPower = 0;
                leftBackPower = 0;
                rightFrontPower = 0;
                rightBackPower = 0;
            }
            if (gamepad2.x) {
                slidePower = slidePower/3;
            }

            if (gamepad2ButtonB && !prevGamepad2ButtonB && !gamepad2.start) {
                controlledServo = !controlledServo;
            }

            if (!controlledServo) {
                if (gamepad2.dpad_down) {
                    servoSamplePos -= 0.005;
                    servoSamplePos = Math.max(servoSamplePos, 0);
                }
                if (gamepad2.dpad_up) {
                    servoSamplePos += 0.005;
                    servoSamplePos = Math.min(servoSamplePos, 0.55);
                }
                if (gamepad2.dpad_right) {
                    servoSamplePos = 0.55;
                }
                if (gamepad2.dpad_left) {
                    servoSamplePos = 0.025;
                }
            } else {
                if (gamepad2.dpad_down) {
                    servoSpecimenPos -= 0.01;
                    servoSpecimenPos = Math.max(servoSpecimenPos, 0.5);
                }
                if (gamepad2.dpad_up) {
                    servoSpecimenPos += 0.01;
                    servoSpecimenPos = Math.min(servoSpecimenPos, 1);
                }
                if (gamepad2.dpad_right) {
                    servoSpecimenPos = 1;
                }
                if (gamepad2.dpad_left) {
                    servoSpecimenPos = 0.5;
                }
            }
            servoSampleClaw.setPosition(servoSamplePos);
            servoSpecimenClaw.setPosition(servoSpecimenPos);

            motorFL.setPower(leftFrontPower);
            motorBL.setPower(leftBackPower);
            motorFR.setPower(rightFrontPower);
            motorBR.setPower(rightBackPower);

            if (gamepad2.y) {
                motorSlide.setTargetPosition(motorSlide.getCurrentPosition());
                motorSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlide.setPower(1);

                while (gamepad2.y && opModeIsActive()) {}
                while (!gamepad2.y && opModeIsActive()) {}
                while (gamepad2.y && opModeIsActive()) {}

                motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorSlide.setPower(0);
            }
            motorSlide.setPower(slidePower);

            if (gamepad2ButtonA & !prevGamepad2ButtonA) {
                armPos = motorArm.getCurrentPosition();
            }
            if (gamepad2ButtonA) {
                motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorArm.setTargetPosition(armPos);
                motorArm.setPower(0.5);
            } else {
                motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorArm.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
            }

            if (gamepad1ButtonA && !prevGamepad1ButtonA && !gamepad1.start) {
                motorBL.setPower(0);
                motorFL.setPower(0);
                motorFR.setPower(0);
                motorBR.setPower(0);
                setSlidePos(SLIDE_SPEED, 1850, 3);
                setArmPos(ARM_SPEED, 435, 1, true);
                setSlidePos(0.5, 1350, 1);
            }

            telemetry.addData("Right Bumper", gamepad2.right_bumper);
            telemetry.addData("Left Bump", gamepad2.left_bumper);
            telemetry.addData("Controlling Servo: ", !controlledServo ? "Sample" : "Specimen");
            telemetry.addData("Sample Servo Position", servoSampleClaw.getPosition());
            telemetry.addData("Specimen Servo Position", servoSpecimenClaw.getPosition());
            telemetry.addData("Arm Mode", motorArm.getMode());
            telemetry.addData("Arm Position", motorArm.getCurrentPosition());
            telemetry.addData("Slide Mode", motorSlide.getMode());
            telemetry.addData("Slide Position", motorSlide.getCurrentPosition());
            telemetry.addData("FR Motor", motorFR.getCurrentPosition());
            telemetry.addData("FL Motor", motorFL.getCurrentPosition());
            telemetry.addData("BR Motor", motorBR.getCurrentPosition());
            telemetry.addData("BL Motor", motorBL.getCurrentPosition());
            telemetry.update();
        }

    }
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

