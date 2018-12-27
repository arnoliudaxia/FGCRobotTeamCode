package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name = "FGCUndefeated", group = "Linear Opmode")
//@Disabled
public class FGCUndefeated extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();//计时
    //region 卷球
    private DcMotor motorCollector;
    private double Power = .8;
    boolean isRunning = false;
    //endregion
    //region 定义闸门舵机
   // private Servo servoGate;
    private Servo servoBlue;
    //private Servo servoOrange;
    //endregion
    //region 闸门角度
    private double BluePosition = 0;
//    private double OrangePosition = 0;
//    private double GatePosition = 0;
    //endregion
    //region 颜色传感器和角度定义
//    private ColorSensor colorSensor;
//    private final int Blue = 12;
//    private final int Orange = 15;
//    //endregion
    //region 舵机开关角度
    private final double BlueOpen = .12;
    private final double BlueClose = 0;
//    private final double OrangeOpen = 0.28;
//    private final double OrangeClose = .5;
//    private final double GateUp = .36;
//    private final double GateDown = .1;
    //endregion
    //region 定义底盘、卷机
    private DcMotor motorLF;
    private DcMotor motorRF;
    private DcMotor motorLB;
    private DcMotor motorRB;
    //endregion
    //region 定义档位
//    double powerMode = 1;//切换快/慢速模式
//    final double POWER_MODE_SLOW = 2.5;
//    final double POWER_MODE_FAST = 1;

    //endregion
    //region 定义抬升电机及其功率
    private DcMotor motorLeftLift;
    private DcMotor motorRightLift;
    private boolean isLiftRunning = false;
    private double LiftPower = .4;

    //endregion
    @Override
    public void init()
    {
//        CollectSystemConfigue();
//        telemetry.addData("国家第一宪兵大队","集合完毕!!!");
//        ControlSystemConfigue();
//        telemetry.addData("战时最高指挥部","已就位!!!");
//        ColourConfigue();
//        telemetry.addData("狼牙侦查连","组织完成!!!");
        PowerSystemConfigure();
        telemetry.addData("第二装甲集团军","整装待发!!!");
//        LiftConfigue();
//        telemetry.addData("空军","发动机已预热");
//        telemetry.addData("电量",getBatteryVoltage());
        telemetry.update();

    }

    //region Not Important
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

    //endregion
    @Override
    public void loop()
    {
//        CollectLoop();
//        SetDegree();
        PowerLoop();
//        LiftLoop();
        DebugGamePad();
        telemetry.update();


    }

    //region 硬件初始化
    private void CollectSystemConfigue()
    {
        //region 实例化卷球电机并设置方向
        motorCollector = hardwareMap.get(DcMotor.class,"motorCollector");
        motorCollector.setDirection(DcMotor.Direction.FORWARD);
        //endregion
    }

    private void ControlSystemConfigue()
    {
        //region 实例化闸门舵机
//        servoGate = hardwareMap.get(Servo.class,"servoGate");
        servoBlue = hardwareMap.get(Servo.class,"servoBlue");
//        servoOrange = hardwareMap.get(Servo.class,"servoOrange");
        //endregion
        //region 初始化闸门角度
        BluePosition = servoBlue.getPosition();
//        OrangePosition = servoOrange.getPosition();
//        GatePosition = servoGate.getPosition();
        //endregion
    }

    private void ColourConfigue()
    {
        //实例化颜色传感器
//        colorSensor = hardwareMap.get(ColorSensor.class,"colour");
    }

    private void PowerSystemConfigure()
    {
        //region 实例化电机
        motorLF = hardwareMap.get(DcMotor.class,"motorLF");
        motorRF = hardwareMap.get(DcMotor.class,"motorRF");
        motorLB = hardwareMap.get(DcMotor.class,"motorLB");
        motorRB = hardwareMap.get(DcMotor.class,"motorRB");
//        motorLeftLift = hardwareMap.get(DcMotor.class,"motorLeftLift");
//        motorRightLift = hardwareMap.get(DcMotor.class,"motorRightLift");
        //endregion
        //region 设置电机转向
        motorLF.setDirection(DcMotor.Direction.REVERSE);
        motorLB.setDirection(DcMotor.Direction.FORWARD);
        motorRF.setDirection(DcMotor.Direction.FORWARD);
        motorRB.setDirection(DcMotor.Direction.REVERSE);
        //endregion

    }

    private void LiftConfigue()
    {
        //region 实例化抬升电机并设置转向
        motorLeftLift = hardwareMap.get(DcMotor.class,"motorLeftLift");
        motorRightLift = hardwareMap.get(DcMotor.class,"motorRightLift");
        motorLeftLift.setDirection(DcMotor.Direction.REVERSE);
        motorRightLift.setDirection(DcMotor.Direction.FORWARD);
        //endregion
    }
    //endregion

    //region 卷球控制
    private void CollectLoop()
    {
        if(gamepad1.left_bumper)
        {
            motorCollector.setPower(Power);
        } else if(gamepad1.right_bumper)
        {
            motorCollector.setPower(-Power);
        }
        if(gamepad1.x)
        {
            Power = (Power==0.8)?.5:.8;
            if(!(motorCollector.getPower()==0))
            {
             motorCollector.setPower(Power);
            }
        }
        telemetry.addData("CollectPower",motorCollector.getPower());
        //↓已注释
        //region 测试卷球功率
        //        if (gamepad1.left_bumper)
//        {
//            while (gamepad1.left_bumper){}
//            LiftPower= Range.clip( motorCollector.getPower()+.1,-1,1);
//
//        }
//        if (gamepad1.right_bumper)
//        {
//            while (gamepad1.right_bumper){}
//            LiftPower= Range.clip( motorCollector.getPower()-.1,-1,1);
//        }
//        telemetry.addData("CollectPower",LiftPower);
//       motorCollector.setPower(LiftPower);
//        telemetry.update();
        //endregion()
    }

    public void Collecting()
    {
        motorCollector.setPower(.5);
    }

    public void StopCollecting(){motorCollector.setPower(0);}


    //endregion
    //region 舵机控制
    private void SetDegree()
    {
        if(gamepad1.dpad_down || gamepad1.dpad_up  )
        {

            //region 根据手柄1的方向轮盘调整红蓝球门的角度
            if(gamepad1.dpad_up)
            {
                //noinspection StatementWithEmptyBody
                while(gamepad1.dpad_up)
                {
                }
                //sleep(50);
                BluePosition = BlueOpen;
            }
            if(gamepad1.dpad_down)
            {
                //noinspection StatementWithEmptyBody
                while(gamepad1.dpad_down)
                {
                }
                //sleep(50);
                BluePosition = BlueClose;
            }
//            if(gamepad1.dpad_left)
//            {
//                //noinspection StatementWithEmptyBody
//                while(gamepad1.dpad_left)
//                {
//                }
//                //sleep(50);
//                OrangePosition = OrangeOpen;
//            }
//
//            if(gamepad1.dpad_right)
//            {
//                //noinspection StatementWithEmptyBody
//                while(gamepad1.dpad_right)
//                {
//                }
//                //sleep(50);
//                OrangePosition = OrangeClose;
//            }
//            if (gamepad1.y)
//            {
//                //noinspection StatementWithEmptyBody
//                while (gamepad1.y){}
//                GatePosition=Range.clip(GatePosition+.02,0,1);
//            }
//            if (gamepad1.a)
//            {
//                //noinspection StatementWithEmptyBody
//                while (gamepad1.a){}
//                GatePosition=Range.clip(GatePosition-.02,0,1);
//
//            }

            //endregion
//            servoOrange.setPosition(OrangePosition);
            servoBlue.setPosition(BluePosition);

        }
        //region 由颜色传感器的数据实时调整
//        if(gamepad2.y)
//        {
//            while(gamepad2.y){}
//            servoGate.setPosition(GateUp);
//        } else if(gamepad2.a)
//        {
//            while(gamepad2.a){}
//
//            servoGate.setPosition(GateDown);
//        }
        //endregion
    }

    //endregion
    //region 动力系统
    private void PowerLoop()
    {
        if(gamepad1.left_stick_x!=0)
        {
            if(gamepad1.left_stick_x < 0)
            {
                moveFix(.9,1,FGCpower.moveDirection.L);
            } else
            {
                moveFix(.9,1,FGCpower.moveDirection.R);
            }
        }
        if(gamepad1.right_stick_y!=0)
        {
            if(gamepad1.right_stick_y < 0)
                moveFix(1,FGCpower.moveDirection.F);
            else
                moveFix(1,FGCpower.moveDirection.B);
        } else
        {
            PowerStop();

        }
//        telemetry.addData("y",gamepad1.left_stick_y);
//        `   telemetry.addData("x",gamepad1.left_stick_x);
    }

    private void PowerStop()
    {
        moveVar(0,0,1);
        moveFix(0,FGCpower.moveDirection.F);
    }

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

    private void moveFix(double power,FGCpower.moveDirection moveDirection)
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

    private void moveFix(double powerF,double powerB,FGCpower.moveDirection Direction)
    {
        switch(Direction)
        {
            case F:
                motorLF.setPower(powerF);
                motorRF.setPower(powerF);
                motorLB.setPower(powerB);
                motorRB.setPower(powerB);
            case B:
                motorLF.setPower(-powerF);
                motorRF.setPower(-powerF);
                motorLB.setPower(-powerB);
                motorRB.setPower(-powerB);
            case L:
                motorLF.setPower(-powerF);
                motorRF.setPower(powerF);
                motorLB.setPower(-powerB);
                motorRB.setPower(powerB);
                break;
            case R:
                motorLF.setPower(powerF);
                motorRF.setPower(-powerF);
                motorLB.setPower(powerB);
                motorRB.setPower(-powerB);

        }
    }

    enum moveDirection
    {
        F,B,R,L
    }

    //endregion
    //region 升降系统
    public void LiftLoop()
    {
        if (gamepad1.y) {
            while(gamepad1.y){}
            motorLeftLift.setPower(LiftPower);
            motorRightLift.setPower(LiftPower);
            isLiftRunning =true;
        }
        if (gamepad1.b)
        {
            while (gamepad1.b){}
            motorLeftLift.setPower(0);
            motorRightLift.setPower(0);
            isLiftRunning =false;
        }
        if (!gamepad1.start&&gamepad1.a)
        {
            while(gamepad1.a){}
            motorLeftLift.setPower(-.6);
            motorRightLift.setPower(-.6);
            isLiftRunning =true;
        }
    }
    //endregion
    //region 电量
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
    //endregion
    private void DebugGamePad()
    {
        if(gamepad1.a) telemetry.addData("按下键:","A");
        if(gamepad1.b) telemetry.addData("按下键:","B");
        if(gamepad1.x) telemetry.addData("按下键:","X");
        if(gamepad1.y) telemetry.addData("按下键:","Y");
        if(gamepad1.start) telemetry.addData("按下键:","Start");
        if(gamepad1.dpad_left) telemetry.addData("按下键:","左");
        if(gamepad1.dpad_right) telemetry.addData("按下键:","右");
        if(gamepad1.dpad_up) telemetry.addData("按下键:","上");
        if(gamepad1.dpad_down) telemetry.addData("按下键:","下");
        if(gamepad1.back) telemetry.addData("按下键:","Back");
        if(gamepad1.left_bumper) telemetry.addData("按下键:","LB");
        if(gamepad1.right_bumper) telemetry.addData("按下键:","RB");
    }

}
