#region Guidance
class Guidance_Computer
{

    const double time_between_run_sec = 0.6d;
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
        solution_1 /= (2 * acceleration);
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

public Program() { }

public void Save() { }

public void Main(string argv) { }