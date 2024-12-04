package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="autoLeft3")
public class autoLeft3 extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBR;
    private DcMotor motorBL;
    private DcMotor motorArm;
    private DcMotor motorSlide;
    private Servo servoClaw;

    private ElapsedTime     runtime = new ElapsedTime();

    static final double COUNTS_PER_INCH = 44.62;
    static final double DRIVE_SPEED = 0.4;
    static  final double ARM_SPEED = 0.4;
    static final double SLIDE_SPEED = 0.4;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        motorSlide = hardwareMap.get(DcMotor.class, "motorSlide");
        servoClaw = hardwareMap.get(Servo.class, "servoClaw");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorArm.setDirection(DcMotor.Direction.FORWARD);
        motorSlide.setDirection(DcMotor.Direction.FORWARD);
        servoClaw.setDirection(Servo.Direction.FORWARD);

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

        // Wait for the game to start (driver presses START)
        waitForStart();

        servoClaw.setPosition(0.48);
        tankDrive(0.3, 4.5, 4.5, 1);
        sideDrive(DRIVE_SPEED, 27, 2);
        tankDrive(DRIVE_SPEED,  20,  20, 2);
        setArmPos(ARM_SPEED, 400, 2, true);
        setSlidePos(SLIDE_SPEED, 2000, 3);
        setArmPos(ARM_SPEED, 450, 1, true);
        setSlidePos(SLIDE_SPEED, 0, 3);
        tankDrive(DRIVE_SPEED, -5, -5, 1);
        setArmPos(ARM_SPEED, 50, 2, false);
        sideDrive(DRIVE_SPEED, -33, 2);
        tankDrive(DRIVE_SPEED, 30, 30, 2);
        tankDrive(DRIVE_SPEED, -22, 22, 1);
        setArmPos(0.25, 980, 2, false);
        tankDrive(0.25, -10, -10, 1);

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
    public void tankDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newFLtarget;
        int newBLtarget;
        int newFRtarget;
        int newBRtarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFLtarget = motorFL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBLtarget = motorBL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFRtarget = motorFR.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBRtarget = motorBR.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
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

            sleep(250);   // optional pause after each move.
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
            newFLtarget = motorFL.getCurrentPosition() + (int)(sideDistance * COUNTS_PER_INCH);
            newBLtarget = motorBL.getCurrentPosition() - (int)(sideDistance * COUNTS_PER_INCH);
            newFRtarget = motorFR.getCurrentPosition() - (int)(sideDistance * COUNTS_PER_INCH);
            newBRtarget = motorBR.getCurrentPosition() + (int)(sideDistance * COUNTS_PER_INCH);
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

            sleep(250);   // optional pause after each move.
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

            sleep(250);   // optional pause after each move.
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

            sleep(250);   // optional pause after each move.
        }
    }
}