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


    private  bool IsZero(ref Vector3D v, double epsilon = 1e-4)
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
        double Bearing = angle_between( forwardProjPlaneVec,  relativeNorthVec);

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



IMyRemoteControl remote_control;
IMyWarhead ref_warhead;
IMyProgrammableBlock ref_sideways;

Compass meow;


public Program()
{
    remote_control = GridTerminalSystem.GetBlockWithName("AGG-7-1 Remote Control") as IMyRemoteControl;
    ref_warhead = GridTerminalSystem.GetBlockWithName("AGG-7-1 Warhead FRONT") as IMyWarhead;
    ref_sideways = GridTerminalSystem.GetBlockWithName("AGG-7-1 Programmable Block") as IMyProgrammableBlock;

    if (remote_control == null || gyro == null || ref_warhead == null || ref_sideways == null)
    {

        Echo("failed to get blocks");

    }
    else
    {
        meow = new Compass(remote_control, ref_warhead, ref_sideways);
    }

    Runtime.UpdateFrequency = UpdateFrequency.Update10;

    Echo("inited");

}

public void Save() { }

public void Main(string argv)
{
    meow.calc_attitude();
    Echo(meow.Yaw.ToString());
    //Echo(remote_control.get);
}