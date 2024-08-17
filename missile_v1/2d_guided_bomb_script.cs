#region Prelude
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using System;
using System.Linq;
using VRageMath;
using System.Text;
using SpaceEngineers.CommonLibs;

namespace SpaceEngineers.UWBlockPrograms.SampleMainNs
{
    public sealed class Program : MyGridProgram
    {
        #endregion
        // Variables

        const string wpn_name = "AGG-7";
        const string wpn_identifier = "-1 ";

        const string Hardpoint_name = "";
        const int distance_to_arm = 20;
        const int distance_to_detonate = 0;

        double[] attitude_setpoint = { 0f, -999, 0f };
        float[] pitch_limit = { 0.5f, -0.5f };


        PID roll_PID = new PID(1f, 0f, 0f, 1f / 6f);
        PID pitch_PID = new PID(1f, 0f, 0f, 1f / 6f);
        PID yaw_PID = new PID(1f, 0f, 0f, 1f / 6f);


        Compass Attitude_Calculator;
        VelocityCompass Vertical_Velocity_Compass;
        Guidance_Computer Guidance_Calculator;

        IMyMotorRotor Hardpoint;

        IMyRemoteControl RemoteControl;
        IMyGyro Gyro;
        IMyTerminalBlock ForwardRef;
        IMyTerminalBlock RightRef;

        IMyMotorRotor WingRotator;
        List<IMyWarhead> Payload = new List<IMyWarhead>();

        private void deploy_wings() { }





        private bool able_to_be_launched = false;

        private bool launched = false;
        private bool explosive_payload;
        private bool wings_deployed = false;
        private bool terminal_stage = false;

        void launch_setup()
        {
            GridTerminalSystem.GetBlocksOfType(Payload) as IMyWarhead;
            if (Payload == null)
            {
                explosive_payload = false;

            }
            else
            {
                explosive_payload = true;
                foreach (IMyWarhead warhead in Payload)
                {
                    warhead.IsArmed = false;
                }
                WingRotator.SetValue("Force weld", false);
            }
            launched = true;
        }

        private int _tickCount = 0;
        public Program()
        {
            //Echo = text => { };

            Echo("begin setup!");


            RemoteControl = GridTerminalSystem.GetBlockWithName(wpn_name + wpn_identifier + "Remote Control") as IMyRemoteControl;
            Gyro = GridTerminalSystem.GetBlockWithName(wpn_name + wpn_identifier + "Gyroscope") as IMyGyro;
            ForwardRef = GridTerminalSystem.GetBlockWithName(wpn_name + wpn_identifier + "Warhead FRONT") as IMyTerminalBlock;
            RightRef = GridTerminalSystem.GetBlockWithName(wpn_name + wpn_identifier + "Programmable Block") as IMyTerminalBlock;

            WingRotator = GridTerminalSystem.GetBlockWithName(wpn_name + wpn_identifier + "Wing Rotator Rotor") as IMyMotorRotor;


            if (RemoteControl == null || Gyro == null || ForwardRef == null || RightRef == null)
            {
                Echo("failed to get blocks");
            }
            else
            {
                Runtime.UpdateFrequency = UpdateFrequency.Update10;

                Gyro.GyroOverride = true;
                Gyro.Roll = 0;
                Gyro.Pitch = 0;
                Gyro.Yaw = 0;

                Attitude_Calculator = new Compass(RemoteControl, ForwardRef, RightRef);
                Attitude_Calculator.calc_attitude();

                Vertical_Velocity_Compass = new VelocityCompass(RemoteControl);
                Vertical_Velocity_Compass.calc_Velocity();

                Guidance_Calculator = new Guidance_Computer(RemoteControl);

                set_Yaw_to_forward();

                Echo("end setup!");
            }
        }

        public void Save() { }

        public void Main(string argv)
        {
            _tickCount++;

            if (launched)
            {

                Attitude_Calculator.calc_attitude();
                Vertical_Velocity_Compass.calc_Velocity();

                // Gyro.Yaw = (float)(roll_PID.Control(Attitude_Calculator.Roll - attitude_setpoint[0]));
                Gyro.Roll = (float)yaw_PID.Control(Attitude_Calculator.Yaw - attitude_setpoint[2]);

                Echo(Vertical_Velocity_Compass.Vertical_Speed.ToString());
                Echo(rad_to_deg(Attitude_Calculator.Pitch).ToString());

                if (pitch_limit[0] < Attitude_Calculator.Pitch)
                {
                    //pitch above limit/too high
                    Gyro.Pitch = -0.5f;
                    Echo("pitch too high");
                }
                else if (pitch_limit[1] > Attitude_Calculator.Pitch)
                {
                    //pitch smaller than low limit/too low
                    Gyro.Pitch = 0.5f;
                    Echo("pitch too low");
                }
                else
                {
                    //pith in limits
                    Gyro.Pitch = (float)pitch_PID.Control(Vertical_Velocity_Compass.Vertical_Speed - velocity_setpoint[1]) / 100f;
                    //Gyro.Pitch = (float)(pitch_PID.Control(Attitude_Calculator.Pitch - attitude_setpoint[1]));

                    Echo("regulated");
                }

                if (!wings_deployed)
                {
                    if (_tickCount > 5)
                    {
                        WingRotator.SetValue("Force weld", true);
                        wings_deployed = true;
                    }
                }

                if (explosive_payload)
                {
                    if (!terminal_stage && Guidance_Calculator.get_distance_to_target() <= distance_to_arm)
                    {
                        if (!terminal_stage)
                        {
                            terminal_stage = true;
                            foreach (IMyWarhead warhead in Payload)
                            {
                                warhead.IsArmed = true;
                            }
                        }
                    }

                    if (terminal_stage && Guidance_Calculator.get_distance_to_target() <= distance_to_detonate)
                    {
                        foreach (IMyWarhead warhead in Payload)
                        {
                            warhead.Detonate();
                        }
                    }
                }
            }
            else
            {
                if (argv == "launch")
                {
                    if (able_to_be_launched)
                    {
                        launch_setup();
                    }
                    else
                    {
                        Echo("cannot be launched");
                    }
                }
                else if (argv != "")
                {
                    Vector3D target_pos;

                    try
                    {
                        target_pos = Vector3D.Parse(argv);
                        Guidance_Calculator.set_target(Vector3D.Vector3D.Parse(argv));
                        able_to_be_launched = true;
                    }
                    catch (Exception e)
                    {
                        Echo("failed to parse arg");
                        Echo(e.ToString());
                    }
                }
            }

        }
        #region PreludeFooter
    }
}
#endregion


#region 3D Compass
class Compass
{
    private Vector3D forward;
    private Vector3D east;

    private Vector3D grav;

    public double Roll { get; private set; } = 0;
    public double Pitch { get; private set; } = 0;
    public double Yaw { get; private set; } = 0;

    private IMyRemoteControl _m_remoteControl;
    private IMyTerminalBlock _m_forward_ref;
    private IMyTerminalBlock _m_right_ref;

    public Compass(IMyRemoteControl reference, IMyTerminalBlock forward_ref, IMyTerminalBlock right_ref)
    {
        _m_remoteControl = reference;


        _m_forward_ref = forward_ref;
        _m_right_ref = right_ref;
    }

    private double angle_between(Vector3D vector1, Vector3D vector2)
    {
        double angle = Vector3D.Dot(vector1, vector2);
        angle /= (vector1.Length() * vector2.Length());
        angle = Math.Acos(angle);

        return angle;
    }


    private bool IsZero(ref Vector3D v, double epsilon = 1e-4)
    {
        if (Math.Abs(v.X) > epsilon) return false;
        if (Math.Abs(v.Y) > epsilon) return false;
        if (Math.Abs(v.Z) > epsilon) return false;
        return true;
    }

    private void Projection(ref Vector3D a, ref Vector3D b, out Vector3D result)
    {
        if (IsZero(ref a) || IsZero(ref b))
        {
            result = Vector3D.Zero;
            return;
        }

        double dot;
        if (Vector3D.IsUnit(ref b))
        {
            Vector3D.Dot(ref a, ref b, out dot);
            Vector3D.Multiply(ref b, dot, out result);
            return;
        }

        double lenSq;
        Vector3D.Dot(ref a, ref b, out dot);
        lenSq = b.LengthSquared();
        Vector3D.Multiply(ref b, dot / lenSq, out result);
    }


    public void get_forward_vector()
    {
        forward = Vector3D.Subtract(_m_forward_ref.GetPosition(), _m_remoteControl.GetPosition());
        forward.Normalize();
    }

    public void get_sideways_vector()
    {
        east = Vector3D.Subtract(_m_right_ref.GetPosition(), _m_remoteControl.GetPosition());
        east.Normalize();
    }



    public double get_roll()
    {
        return angle_between(east, grav) - 1.5708d;
    }

    public double get_pitch()
    {
        return angle_between(forward, grav) - 1.5708d;
    }

    public double get_yaw()
    {
        Vector3D northv = new Vector3D(0, -1, 0);

        Vector3D relativeEastVec = grav.Cross(northv);


        Vector3D relativeNorthVec;

        Vector3D.Cross(ref relativeEastVec, ref grav, out relativeNorthVec);

        //project forward vector onto a plane comprised of the north and east vectors 
        Vector3D forwardProjNorthVec;
        Projection(ref forward, ref relativeNorthVec, out forwardProjNorthVec);
        Vector3D forwardProjEastVec;
        Projection(ref forward, ref relativeEastVec, out forwardProjEastVec);
        Vector3D forwardProjPlaneVec = forwardProjEastVec + forwardProjNorthVec;

        //find angle from abs north to projected forward vector measured clockwise 
        double Bearing = angle_between(forwardProjPlaneVec, relativeNorthVec);

        return Bearing;
    }


    public void calc_attitude()
    {
        grav = _m_remoteControl.GetNaturalGravity();

        get_forward_vector();
        get_sideways_vector();

        Roll = get_roll();
        Pitch = get_pitch();
        Yaw = get_yaw();

    }
}
#endregion


#region Velocity Compass
class VelocityCompass
{
    public Vector3D Vertical_Velocity_global_frame;
    public double Vertical_Speed;
    public Vector3D Global_Velocity;

    private Vector3D grav;
    private Vector3D old_position;


    private IMyRemoteControl _m_remoteControl;


    public VelocityCompass(IMyRemoteControl reference)
    {
        _m_remoteControl = reference;

    }

    private void get_Global_Velocity()
    {
        const double error_val = 0.001;
        Vector3D current_pos = _m_remoteControl.GetPosition();

        if (Math.Abs(current_pos.X - old_position.X) >= error_val || Math.Abs(current_pos.Y - old_position.Y) >= error_val || Math.Abs(current_pos.Z - old_position.Z) >= error_val)
        {
            Global_Velocity = Vector3D.Subtract(current_pos, old_position);
            Global_Velocity.Normalize();
            Global_Velocity = Global_Velocity * _m_remoteControl.GetShipSpeed();

            old_position = current_pos;

        }
    }

    private void get_Vertical_Velocity()
    {
        grav = _m_remoteControl.GetNaturalGravity();
        grav.Normalize();


        double proj_velocity = Vector3D.Dot(grav, Global_Velocity);
        proj_velocity /= Math.Pow(grav.Length(), 2);
        Vector3D projection = grav * proj_velocity;
        Vertical_Velocity_global_frame = projection;

        if (Vector3D.Dot(Vertical_Velocity_global_frame, grav) < 0)
        {
            Vertical_Speed = Vertical_Velocity_global_frame.Length();
        }
        else
        {
            Vertical_Speed = -Vertical_Velocity_global_frame.Length();
        }
        Vertical_Speed = Math.Round(Vertical_Speed, 2);
    }

    public void calc_Velocity()
    {
        get_Global_Velocity();
        get_Vertical_Velocity();
    }
}
#endregion


#region Guidance Computer
class Guidance_Computer
{

    const float time_between_run_sec = 0.6f;
    private Vector3D target_position;
    private Vector3D current_position;
    public Vector2D target_vector;

    private Vector3D grav;

    private double old_vertical_velocity = 0;
    private double vertical_acceleration = 0;

    private IMyRemoteControl _m_remoteControl;



    public double get_distance_to_target()
    {
        get_target_vector();
        return target_vector.Length();
    }

    private double Time_of_Flight(double displacement, double velocity, double acceleration)
    {
        double top = Math.Pow(velocity, 2) + 2 * displacement * acceleration;
        top = Math.Sqrt(top);

        double solution_1 = -velocity + top;
        solution_1 /= 2 * acceleration;
        if (solution_1 > 0)
        {
            return solution_1;
        }
        else
        {
            double solution_2 = -velocity - top;
            solution_2 /= (2 * acceleration);
            return solution_2;
        }
    }

    public Guidance_Computer(IMyRemoteControl reference)
    {
        _m_remoteControl = reference;
    }

    public void set_target(Vector3D target_GPS)
    {
        target_position = target_GPS;
    }

    private void get_target_vector()
    {
        grav = _m_remoteControl.GetNaturalGravity();

        Vector3D target_vector_3D = Vector3D.Subtract(current_position, target_position);

        double proj = Vector3D.Dot(grav, target_vector_3D);

        proj /= Math.Pow(grav.Length(), 2);

        target_vector.Y = proj;
        target_vector.X = Math.Sqrt(target_vector_3D.LengthSquared() - Math.Pow(proj, 2));
    }

    public double calc_vertical_TOF(double vertical_velocity, double vertical_acceleration)
    {
        get_target_vector();

        return Time_of_Flight(target_vector.Y, -vertical_velocity, vertical_acceleration);
    }

    public double calculate_horizontal_TOF(double horizontal_velocity, double horizontal_acceleration)
    {
        get_target_vector();

        return Time_of_Flight(target_vector.X, horizontal_velocity, horizontal_acceleration);

    }
}
#endregion

#region PID Class

/// <summary>
/// Discrete time PID controller class.
/// Last edited: 2022/08/11 - Whiplash141
/// </summary>
public class PID
{
    public double Kp { get; set; } = 0;
    public double Ki { get; set; } = 0;
    public double Kd { get; set; } = 0;
    public double Value { get; private set; }

    double _timeStep = 0;
    double _inverseTimeStep = 0;
    double _errorSum = 0;
    double _lastError = 0;
    bool _firstRun = true;

    public PID(double kp, double ki, double kd, double timeStep)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        _timeStep = timeStep;
        _inverseTimeStep = 1 / _timeStep;
    }

    protected virtual double GetIntegral(double currentError, double errorSum, double timeStep)
    {
        return errorSum + currentError * timeStep;
    }

    public double Control(double error)
    {
        //Compute derivative term
        double errorDerivative = (error - _lastError) * _inverseTimeStep;

        if (_firstRun)
        {
            errorDerivative = 0;
            _firstRun = false;
        }

        //Get error sum
        _errorSum = GetIntegral(error, _errorSum, _timeStep);

        //Store this error as last error
        _lastError = error;

        //Construct output
        Value = Kp * error + Ki * _errorSum + Kd * errorDerivative;
        return Value;
    }

    public double Control(double error, double timeStep)
    {
        if (timeStep != _timeStep)
        {
            _timeStep = timeStep;
            _inverseTimeStep = 1 / _timeStep;
        }
        return Control(error);
    }

    public virtual void Reset()
    {
        _errorSum = 0;
        _lastError = 0;
        _firstRun = true;
    }
}

public class DecayingIntegralPID : PID
{
    public double IntegralDecayRatio { get; set; }

    public DecayingIntegralPID(double kp, double ki, double kd, double timeStep, double decayRatio) : base(kp, ki, kd, timeStep)
    {
        IntegralDecayRatio = decayRatio;
    }

    protected override double GetIntegral(double currentError, double errorSum, double timeStep)
    {
        return errorSum * (1.0 - IntegralDecayRatio) + currentError * timeStep;
    }
}
#endregion