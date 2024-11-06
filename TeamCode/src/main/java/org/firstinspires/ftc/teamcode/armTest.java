package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "armTest")
public class armTest extends LinearOpMode{
    public DcMotor motorArm;
    public DcMotor motorSlide;
    public Servo servoClaw;
    //    public DcMotor[] motors = new DcMotor[4];


    public double servoPos = 0.2;
    public int armPos = 0;
    public double armPower = 0;

    // motorArm - Min: 0 Max: 950
    // motorSlide - Min:  Max:

    public void runOpMode() throws InterruptedException {
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        motorSlide = hardwareMap.get(DcMotor.class, "motorSlide");
        servoClaw = hardwareMap.get(Servo.class, "servoClaw");

        motorSlide.setDirection(DcMotor.Direction.FORWARD);
        servoClaw.setDirection(Servo.Direction.FORWARD);

        motorSlide.setPower(armPower);
        motorArm.setPower(0);
        motorArm.setTargetPosition(armPos);
        servoClaw.setPosition(servoPos);

        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            armPos += (int) (10 * (gamepad2.right_trigger - gamepad2.left_trigger));
            double slidePower = ((gamepad2.left_bumper ? 1 : 0) - (gamepad2.right_bumper ? 1 : 0));

            if (gamepad2.dpad_up) {
                servoPos -= 0.0005;
            }
            if (gamepad2.dpad_down) {
                servoPos += 0.0005;
            }
            servoClaw.setPosition(servoPos);
            motorArm.setTargetPosition(armPos);
            motorArm.setPower(0.5);
            motorSlide.setPower(slidePower);

            telemetry.addData("Arm Power", "%.2f", (gamepad2.right_trigger - gamepad2.left_trigger));
            telemetry.addData("Target Position", armPos);
            telemetry.addData("Arm Position", motorArm.getCurrentPosition());
            telemetry.addData("Slide Position", motorSlide.getCurrentPosition());
            telemetry.update();
        }

    }
}

