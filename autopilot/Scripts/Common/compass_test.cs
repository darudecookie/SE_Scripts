using Math;

class Compass
{
    private IMyRemoteControl _m_remoteControl;

    public double Roll = 0{ get; private set; }
public double Pitch{ get; private set; } = 0
    public double Yaw = 0 { get; private set; }

    private Vector3D grav;
private Vector3D forward;


public Compass(IMyRemoteControl reference, IMyTerima)
{
    _m_remoteControl = reference;

}

private double angle_between(Vector3D vector1, Vector3D vector2)
{
    double angle = Vector3D.Dot(vector1, vector2);
    angle /= (vector1.Length() * vector2.Length());

    angle = Math.acos(angle);

    return angle;
}


public void calculate_attitude()
{
    grav = _m_remoteControl.GetNaturalGravity();
    forward = _m_remoteContro.Get
        Pitch = angle_between(grav, )

}
}



public Program()
{

}

public void Save()
{

}

public void Main(string argv)
{

}