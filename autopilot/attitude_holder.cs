double[] attitude_setpoint = { 0f, 0.1f, 0f };
double[] velocity_setpoint = { 0f, 0f, 0f };


float[] pitch_limit = { 0.5f, -0.5f };



PID roll_PID = new PID(1f, 0f, 0f, 1f / 6f);
PID pitch_PID = new PID(1f, 0f, 0f, 1f / 6f);
PID yaw_PID = new PID(1f, 0f, 0f, 1f / 6f);


Compass Attitude_Calculator;
VelocityCompass Vertical_Velocity_Compass;


IMyRemoteControl RemoteControl;
IMyGyro Gyro;
IMyTerminalBlock ForwardRef;
IMyTerminalBlock RightRef;

double rad_to_deg(double rad)
{
    return rad * (180d / Math.PI);
}


void set_Yaw_to_forward()
{
    Attitude_Calculator.calc_attitude();
    attitude_setpoint[2] = Attitude_Calculator.Yaw;
}


bool setup_complete = false;
public Program()
{
    //Echo = text => { };

    Echo("begin setup!");
    Runtime.UpdateFrequency = UpdateFrequency.Update10;


    RemoteControl = GridTerminalSystem.GetBlockWithName("AGG-7-1 Remote Control") as IMyRemoteControl;
    Gyro = GridTerminalSystem.GetBlockWithName("AGG-7-1 Gyroscope") as IMyGyro;
    ForwardRef = GridTerminalSystem.GetBlockWithName("AGG-7-1 Warhead FRONT") as IMyTerminalBlock;
    RightRef = GridTerminalSystem.GetBlockWithName("AGG-7-1 Programmable Block") as IMyTerminalBlock;

    if (RemoteControl == null || Gyro == null || ForwardRef == null || RightRef == null)
    {

        Echo("failed to get blocks");

    }
    else
    {
        setup_complete = true;

        Gyro.GyroOverride = true;
        Gyro.Roll = 0;
        Gyro.Pitch = 0;
        Gyro.Yaw = 0;

        Attitude_Calculator = new Compass(RemoteControl, ForwardRef, RightRef);
        Attitude_Calculator.calc_attitude();

        Vertical_Velocity_Compass = new VelocityCompass(RemoteControl);
        Vertical_Velocity_Compass.calc_Velocity();

        set_Yaw_to_forward();

        Echo("end setup!");
    }

}

public void Save() { }

public void Main(string argv)
{
    if (setup_complete)
    {

        Attitude_Calculator.calc_attitude();
        Vertical_Velocity_Compass.calc_Velocity();

       // Gyro.Yaw = (float)(roll_PID.Control(Attitude_Calculator.Roll - attitude_setpoint[0]));
        Gyro.Roll = (float)(yaw_PID.Control(Attitude_Calculator.Yaw - attitude_setpoint[2]));

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
            Gyro.Pitch = (float)(pitch_PID.Control(Vertical_Velocity_Compass.Vertical_Speed - velocity_setpoint[1])) / 100f;
            //Gyro.Pitch = (float)(pitch_PID.Control(Attitude_Calculator.Pitch - attitude_setpoint[1]));

            Echo("regulated");

        }
    }
}



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

        return Bearing - 1.5708d;
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