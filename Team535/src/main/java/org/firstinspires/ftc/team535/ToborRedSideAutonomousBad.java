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

package org.firstinspires.ftc.team535;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


@Autonomous(name="Red Auto Bad", group="Autonomous")
//@Disabled
public class ToborRedSideAutonomousBad extends OpMode
{
    HardwareTOBOR robo = new HardwareTOBOR();
    double heading;
    HardwareTOBOR.direction dir;
    public enum state{
        READJEWEL,
        HITJEWELOUT,
        HITJEWELIN,
        DRIVEFORWARD,
        DRIVEOFFSTONEEQUIVALENT,
        SEEKCOLUMN,
        LEFT,
        CENTER,
        MOVEFORWARD,
        PLACEBLOCK,
        BACKUP,
        STOPALL
    }

    double TPI = 43;
    state currentState = state.DRIVEOFFSTONEEQUIVALENT;
    private RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
    
    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");
        robo.initRobo(hardwareMap);
        robo.initVuforia();
        robo.startVuforia();

        
    }


    @Override
    public void init_loop()
    {
        if (robo.readKey() != RelicRecoveryVuMark.UNKNOWN)
        {
            vuMark = robo.readKey();
            telemetry.addData("Vumark Acquired", vuMark);
        }
        robo.BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robo.FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robo.FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robo.BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }



    @Override
    public void start()
    {
        robo.stopVuforia();
        robo.BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robo.BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robo.FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robo.FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
    }

    @Override
    public void loop()
    {
        telemetry.addData("BRMotor", robo.BRMotor.getCurrentPosition());
        switch (currentState){
            case READJEWEL:
                robo.arm(HardwareTOBOR.armPos.Down);
                dir = robo.knockJewel(HardwareTOBOR.color.Red);
                telemetry.addData("Direction", robo.knockJewel(HardwareTOBOR.color.Red));
                if (dir != HardwareTOBOR.direction.Unknown)
                {
                    currentState = state.HITJEWELOUT;
                }
                break;
            case HITJEWELOUT:
                robo.angles = robo.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if (dir == HardwareTOBOR.direction.Left)
                {
                    robo.BRMotor.setPower(0.3);
                    robo.FRMotor.setPower(0.3);
                    robo.BLMotor.setPower(-0.3);
                    robo.FLMotor.setPower(-0.3);
                    if (robo.angles.firstAngle >= 15)
                    {
                        currentState = state.HITJEWELIN;
                    }
                }
                else if (dir == HardwareTOBOR.direction.Right)
                {
                    robo.BRMotor.setPower(-0.3);
                    robo.FRMotor.setPower(-0.3);
                    robo.BLMotor.setPower(0.3);
                    robo.FLMotor.setPower(0.3);
                    if (robo.angles.firstAngle <= -15)
                    {
                        currentState = state.HITJEWELIN;
                    }
                }
                else
                {
                    currentState = state.READJEWEL;
                }
                break;
            case HITJEWELIN:
                if (dir == HardwareTOBOR.direction.Left)
                {
                    robo.BRMotor.setPower(-0.3);
                    robo.FRMotor.setPower(-0.3);
                    robo.BLMotor.setPower(0.3);
                    robo.FLMotor.setPower(0.3);
                    if (robo.angles.firstAngle <=91 &&robo.angles.firstAngle >=89)
                    {
                        currentState = state.DRIVEFORWARD;
                    }
                }
                else if (dir == HardwareTOBOR.direction.Right)
                {
                    robo.BRMotor.setPower(-0.3);
                    robo.FRMotor.setPower(-0.3);
                    robo.BLMotor.setPower(0.3);
                    robo.FLMotor.setPower(0.3);
                    if (robo.angles.firstAngle <=91 &&robo.angles.firstAngle >=89)
                    {
                        currentState = state.DRIVEFORWARD;
                    }
                }
            case DRIVEFORWARD:
                robo.DriveForwardAuto(0.5,90;
                if (robo.rangeSensor.getDistance(DistanceUnit.INCH) <= )


            case DRIVEOFFSTONEEQUIVALENT:
                heading = robo.strafeRightAuto(0.35);
                if (((25*TPI)+robo.BRMotor.getCurrentPosition()< (0.5*TPI))&&(robo.rangeSensor.getDistance(DistanceUnit.INCH) >= 10))
                {
                    currentState = state.SEEKCOLUMN;
                }
            break;
            case SEEKCOLUMN:
                if (vuMark == RelicRecoveryVuMark.RIGHT || vuMark == RelicRecoveryVuMark.UNKNOWN)
                {
                    currentState = state.PLACEBLOCK;
                }
                
                else if (vuMark == RelicRecoveryVuMark.CENTER)
                {
                    currentState = state.CENTER;
                }
                else if (vuMark == RelicRecoveryVuMark.LEFT)
                {
                    currentState = state.LEFT;
                }
                break;
            case CENTER:
               heading = robo.strafeRightAuto(0.35);
               telemetry.addData("Distance", Math.abs((36*TPI)+robo.BRMotor.getCurrentPosition()));
                if (((36*TPI)+robo.BRMotor.getCurrentPosition()< (0.5*TPI))&&(robo.rangeSensor.getDistance(DistanceUnit.INCH) >= 10))
                {
                    currentState = state.PLACEBLOCK;
                }
                break;
            case LEFT:
                heading = robo.strafeRightAuto(0.35);
                if ((46*TPI)+robo.BRMotor.getCurrentPosition()< (0.5*TPI)&&(robo.rangeSensor.getDistance(DistanceUnit.INCH) >= 10))
                {
                    currentState = state.PLACEBLOCK;
                }
                break;
            case MOVEFORWARD:
                robo.DriveForwardAuto(-0.2,0);
                if (robo.rangeSensor.getDistance(DistanceUnit.INCH)<= 9.5)
                {
                    currentState = state.STOPALL;
                }
                
                break;
            case PLACEBLOCK:
                robo.RPlate.setPosition(.08);
                robo.LPlate.setPosition(1);
                robo.BRMotor.setPower(0);
                robo.FRMotor.setPower(0);
                robo.BLMotor.setPower(0);
                robo.FLMotor.setPower(0);
                currentState = state.MOVEFORWARD;
                break;
            case BACKUP:
                robo.DriveForwardAuto(0.2,0);
                if (robo.rangeSensor.getDistance(DistanceUnit.INCH) <= 4)
                {
                    currentState = state.STOPALL;
                }
                break;
            case STOPALL:
                robo.RPlate.setPosition(.81);
                robo.LPlate.setPosition(.27);
                robo.BRMotor.setPower(0);
                robo.FRMotor.setPower(0);
                robo.BLMotor.setPower(0);
                robo.FLMotor.setPower(0);
            
            break;
        }
        telemetry.addData("State", currentState);
        telemetry.addData("1", "heading: " + heading);
    }

    @Override
    public void stop()
    {

    }

}
