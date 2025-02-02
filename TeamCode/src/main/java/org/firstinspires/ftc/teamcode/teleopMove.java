package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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


    public double speedMode = 1;
    public double servoSamplePos = 0;
    public double servoSpecimenPos = 0.5;
    public int armPos = 0;
    boolean gamepad2ButtonA = false;
    boolean gamepad2ButtonY = false;
    boolean gamepad2ButtonB = false;
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
            gamepad2ButtonA = gamepad2.a;
            gamepad2ButtonB = gamepad2.b;
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
                    servoSamplePos = Math.min(servoSamplePos, 0.545);
                }
                if (gamepad2.dpad_right) {
                    servoSamplePos = 0.545;
                }
                if (gamepad2.dpad_left) {
                    servoSamplePos = 0;
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
                telemetry.addData("Erm What the Sigma: ", "Erm WHat the Sigma");
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
}

