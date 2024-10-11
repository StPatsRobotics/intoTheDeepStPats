package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "motorTest")
public class motorTest extends LinearOpMode{
    public DcMotor motor;
    // Motor and variable definitions

    double motorPower = 0;

    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "motorArm");

        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setPower(0);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Runs when you hit "Init" on the robot

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            motorPower = (gamepad1.right_trigger - gamepad1.left_trigger);

            motor.setPower(motorPower);
            // Main code loop
        }

    }
}
