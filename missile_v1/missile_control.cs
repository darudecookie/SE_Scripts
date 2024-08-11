



//block names
const string wpn_name = "AGM-13";
const string wpn_identifier = "-1 ";

const string remote_control_name = wpn_name + wpn_identifier + "Remote Control";
const string gyro_name = wpn_name + wpn_identifier + "Gyroscope";
const string wing_rotator_rotor = wpn_name + wpn_identifier + "Wing Rotor";

const float dist_to_arm = 25; //meters

IMyRemoteControl remote_control;
IMyGyro gyro;
IMyMotorRotor wing_rotator;
List<IMyWarhead> payload = new List<IMyWarhead>();

List<float> target_position = new List<float>();
List<float> current_position = new List<float>();


public Program()
{
    //Echo = text => { };
    Runtime.UpdateFrequency = UpdateFrequency.Update100;

    GridTerminalSystem.GetBlocksOfType(payload);
    foreach (var Warhead in payload)
    {
        Warhead.IsArmed = false;
    }

    remote_control = GridTerminalSystem.GetBlockWithName(remote_control_name) as IMyRemoteControl;
    gyro = GridTerminalSystem.GetBlockWithName(gyro_name) as IMyGyro;
    wing_rotator = GridTerminalSystem.GetBlockWithName(wing_rotator_rotor) as IMyMotorRotor;



}

void launch_setup()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update10;

}

public void Save()
{
}

public void Main(string args)
{
    
}