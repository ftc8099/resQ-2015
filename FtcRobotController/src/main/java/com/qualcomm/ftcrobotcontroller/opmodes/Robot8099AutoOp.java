/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */
public class Robot8099AutoOp extends OpMode {
    /* extreme values for servo ranges */
    final static double SERVO_1_MIN = 0.20;
    final static double SERVO_1_MAX = 0.90;

    static double pivot_motor1_power = 0.6;
    static double pivot_motor2_power = 0.5;
    static long startTime = System.currentTimeMillis();
    private enum RobotPhase {
        INIT,
        LEAVING_BASE,
        PIVOTING,
        NEARING_RAMP,
        FINISHED
    }

    RobotPhase phase = RobotPhase.INIT;
    long startMillis;
    long waitToPivotMillis = 5 /* seconds */ * 1000;
    long startPivotTime;

    long pivotDuration = 1 /* second */ * 1000;
    long endPivotTime;

    long startRampTime;
    long waitToRampMillis = 5 /* seconds */ * 1000;
    long finishTime;

    // position of servos 1 and 2
    double servo_1_position;

    // change in servo positions
    double servo_1_delta = 0.05;

    double motor_3_delta = 0.1;

    // create two servos
    Servo servo_1;

    // create two dc motors
    DcMotor motor_1;
    DcMotor motor_2;
    DcMotor motor_3;

    // create the sensors
    //IrSeekerSensor ir_seeker;
    OpticalDistanceSensor ods_sensor;
    TouchSensor touch_sensor;

    @Override
    public void init() {
        // acquire motors and servos from hardware
        motor_1 = hardwareMap.dcMotor.get("motor_1");
        motor_2 = hardwareMap.dcMotor.get("motor_2");
        motor_3 = hardwareMap.dcMotor.get("motor_3");
        //servo_1 = hardwareMap.servo.get("servo_1");

        // get the sensors
        //ir_seeker = hardwareMap.irSeekerSensor.get("ir_seeker");
        ods_sensor = hardwareMap.opticalDistanceSensor.get("ods_sensor");
        touch_sensor = hardwareMap.touchSensor.get("touch_sensor");

        // reverse motor 1
        motor_1.setDirection(DcMotor.Direction.REVERSE);

        // initialize servo locations
        //servo_1_position = SERVO_1_MIN;
        startMillis = System.currentTimeMillis();
        startPivotTime = startMillis + waitToPivotMillis;
        telemetry.addData("start_time", System.currentTimeMillis());
    }

    /*
       * Code to run when the op mode is first enabled goes here
       * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
       */
    @Override
    public void init_loop()
    {
        startTime = System.currentTimeMillis();
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        /*
        switch (phase) {
            case INIT:
                motor_1.setPower(0.5);
                motor_2.setPower(0.5);
                phase = RobotPhase.LEAVING_BASE;
                // Fall through
            case LEAVING_BASE:
                motor_1.setPower(0.5);
                motor_2.setPower(0.5);
                if (System.currentTimeMillis() > startPivotTime) {
                    // The wait time has elapsed
                    endPivotTime = startPivotTime + pivotDuration;
                    phase = RobotPhase.PIVOTING;
                }
                break;
            case PIVOTING:
                // Look in the SDK documentation to see if DcMotor has useful methods.
                motor_1.setPower(0.75);
                motor_2.setPower(0.5);
                if (System.currentTimeMillis() > endPivotTime)
                startRampTime = System.currentTimeMillis();
                finishTime = startRampTime + waitToRampMillis;
                phase = RobotPhase.NEARING_RAMP;
                break;
            case NEARING_RAMP:
                motor_1.setPower(0.5);
                motor_2.setPower(0.5);

                if (System.currentTimeMillis() > finishTime) {
                    motor_1.setPower(0);
                    motor_2.setPower(0);
                    phase = RobotPhase.FINISHED;
                }

                // Fall through
                // No explicit handling for FINISHED, group with default.
            default:
                motor_1.setPower(0);
                motor_2.setPower(0);
                break; // Either finished or something went wrong.
        }
        */
        //telemetry.addData("time", new Long(System.currentTimeMillis()).toString());
        telemetry.addData("elapse_time", new Long(System.currentTimeMillis() - startTime).toString());

        if (System.currentTimeMillis() > startTime + (5 * 10000 / 3)) {
            motor_1.setPower(0);
            motor_2.setPower(0);
            telemetry.addData("motor_power", 0);
        } else {
            motor_1.setPower(0.5);
            motor_2.setPower(0.5);
            telemetry.addData("motor_power", 0.5);
        }
    }
}
