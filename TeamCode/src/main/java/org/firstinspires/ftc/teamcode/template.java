package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "template")
public class template extends LinearOpMode{
    // Motor and variable definitions

    public void runOpMode() throws InterruptedException {
        // Runs when you hit "Init" on the robot

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            // Main code loop
        }

    }
}
