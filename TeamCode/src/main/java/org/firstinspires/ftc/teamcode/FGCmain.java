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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.*;


@TeleOp(name = "FGCmain", group = "Iterative Opmode")
@Disabled
public class FGCmain extends OpMode
{

    private ElapsedTime runtime = new ElapsedTime();//计时
    //region 定义底盘、卷机、抬升电机
    private DcMotor motorLF;
    private DcMotor motorRF;
    private DcMotor motorLB;
    private DcMotor motorRB;
    private DcMotor motorLeftLift;
    private DcMotor motorRightLift;
    //endregion
    //region 定义档位
    private double powerMode = 1;//切换快/慢速模式
    final double POWER_MODE_SLOW = 2.5;
    final double POWER_MODE_FAST = 1;
    //endregion

    @Override
    public void init()
    {
        PowerSystemConfigure();//初始化硬件
        //CLS.CollectSystemConfigue();
        telemetry.addData("Commander(HardWare)\t","Ready!!!");
        telemetry.addData("Ammo(Useful)\t",getBatteryVoltage());
        telemetry.update();
    }

    @Override
    public void init_loop()
    {
    }


    @Override
    public void start()
    {
        telemetry.clearAll();
        runtime.reset();
    }

    @Override
    public void loop()
    {

        //region MOVE
        frameControl();//控制底盘移动方法
        telemetry.addData("PowerMode:\t",powerMode);
        telemetry.addData("Speed:\t",motorLF.getPower());
        //endregion
        //region Collect
        //CLS.Loop();
        //telemetry.addData("集球器速度:\t",CLS.motorCollector.getPower());
        //endregion
        //region Control
       /* CtrlSystem.ControlSystemConfigue();
        double[] Degree= CtrlSystem.SetDegree();
        telemetry.addData("OrangeDegree:\t",Degree[1]);
        telemetry.addData("BlueDegree:\t",Degree[0]);*/
        //endregion
        telemetry.update();
    }
    private void PowerSystemConfigure()
    {
        //region 实例化电机
        motorLF = hardwareMap.get(DcMotor.class,"motorLF");
        motorRF = hardwareMap.get(DcMotor.class,"motorRF");
        motorLB = hardwareMap.get(DcMotor.class,"motorLB");
        motorRB = hardwareMap.get(DcMotor.class,"motorRB");
        motorLeftLift = hardwareMap.get(DcMotor.class,"motorLeftLift");
        motorRightLift = hardwareMap.get(DcMotor.class,"motorRightLift");
        //endregion
        //region 设置电机转向
        motorLF.setDirection(DcMotor.Direction.REVERSE);
        motorLB.setDirection(DcMotor.Direction.FORWARD);
        motorRF.setDirection(DcMotor.Direction.REVERSE);
        motorRB.setDirection(DcMotor.Direction.REVERSE);
        //endregion
        //region 实例化颜色/距离传感器
        /*sensorColour = hardwareMap.get(ColorSensor.class,"sensorColour");
        sensorDistance = hardwareMap.get(DistanceSensor.class,"sensorColour");*/
        //endregion

    }
    private double getBatteryVoltage()//获取电量
    {
        double result = Double.POSITIVE_INFINITY;
        for(VoltageSensor sensor : hardwareMap.voltageSensor)
        {
            double voltage = sensor.getVoltage();
            if(voltage > 0)
            {
                result = Math.min(result,voltage);
            }
        }
        return result;
    }
    //region 动力系统
    enum moveDirection
    {
        F,B,R,L
    }

    //由定量输入值的底盘移动方法，0<power<1
    private void moveFix(double power,PowerSystem.moveDirection moveDirection)
    {
        switch(moveDirection)
        {
            case F:
                motorLF.setPower(power);
                motorRF.setPower(power);
                motorLB.setPower(power);
                motorRB.setPower(power);
                break;
            case B:
                motorLF.setPower(-power);
                motorRF.setPower(-power);
                motorLB.setPower(-power);
                motorRB.setPower(-power);
                break;
            case L:
                motorLF.setPower(-power);
                motorRF.setPower(power);
                motorLB.setPower(-power);
                motorRB.setPower(power);
                break;
            case R:
                motorLF.setPower(power);
                motorRF.setPower(-power);
                motorLB.setPower(power);
                motorRB.setPower(-power);
        }
    }


    /**
     * @param yPower    y轴功率（前后平移方向上的功率）
     * @param rPower    自转功率（左右转向的功率）
     * @param powerMode 快慢模式，越大越慢
     */
    private void moveVar(double yPower,double rPower,double powerMode)
    {
        double FinalPower1 = Range.clip((yPower+rPower)/powerMode,-1,1);
        double FinalPower2 = Range.clip((yPower-rPower)/powerMode,-1,1);
        double FinalPower3 = Range.clip((yPower+rPower)/powerMode,-1,1);
        double FinalPower4 = Range.clip((yPower-rPower)/powerMode,-1,1);

        motorLF.setPower(FinalPower1);
        motorRF.setPower(FinalPower2);
        motorLB.setPower(FinalPower3);
        motorRB.setPower(FinalPower4);
    }

    //停止
    private void frameStop()
    {
//        motorLF.setPower(0);
//        motorRF.setPower(0);
//        motorLB.setPower(0);
//        motorRB.setPower(0);

        moveVar(0,0,1);

        moveFix(0,PowerSystem.moveDirection.F);
    }

    //变速
    private double switchPowerMode()
    {
        if(gamepad1.left_bumper)
        {
            return powerMode-.25;
        } else if(gamepad1.right_bumper)
        {
            return powerMode+.25;
        }
        return powerMode;
        //可以自己改方法，比如这里是第一操作手按下LB则稍微降速（返回1.5）；按下RB则降速很多（返回值2.5）
    }

    //手柄控制底盘，将前进功率写入moveVar方法的第一个输入值，转向功率写入moveVar方法的第二个输入值，powerMode的值由调用switchPowerMode方法获得
    private void frameControl()
    {
        if(gamepad1.left_stick_y!=0 || gamepad1.left_stick_x!=0 || gamepad1.left_trigger!=0 || gamepad1.right_trigger!=0)
        {
            moveVar(-gamepad1.left_stick_y,-gamepad1.left_stick_x,switchPowerMode());//例如此处手柄控制方法为第一操作手的左摇杆前后左右摇动控制前进后退与转向
            //手柄上摇杆的y轴（前后推动）向前为-1，向后线性增加直至1；x轴（左右推动）向左为-1，向右线性增加至1
            //手柄上的dpad按键、a,b,x,y按键、左右摇杆按钮、Bumper键均为布尔值
            //摇杆和Trigger键为double值，摇杆已解释过，Trigger键不按下为0，按下后，线性增加直至按到底，为1
            //手柄上的这些名词自行百度

        } else
        {
            frameStop();

        }
    }

    //endregion
}
