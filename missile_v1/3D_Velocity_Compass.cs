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

IMyRemoteControl remote_control;

VelocityCompass meow;

public Program()
{

    remote_control = GridTerminalSystem.GetBlockWithName("AGG-7-1 Remote Control") as IMyRemoteControl;
    if (remote_control == null)
    {
        Echo("failed to get blocks");

    }
    else
    {
        meow = new VelocityCompass(remote_control);
    }

    Runtime.UpdateFrequency = UpdateFrequency.Update10;

    Echo("inited");
}

public void Save() { }

public void Main(string argv)
{
    meow.calc_Velocity();

    Echo(meow.Vertical_Velocity_global_frame.ToString());
    Echo(meow.Vertical_Speed.ToString());
    Echo("hi");
    Echo(DateTime.Now.ToString("hh:mm:ss"));
}