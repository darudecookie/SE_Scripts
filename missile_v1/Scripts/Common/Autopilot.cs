using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace SpaceEngineers.CommonLibs
{
    class RemoteControlPhysicalInfo
    {
        IMyRemoteControl _remoteControl;

        public MatrixD WorldMatrix { get { return _remoteControl.WorldMatrix; } }
        public Vector3D Position { get { return _remoteControl.GetPosition(); } }
        public Vector3D GravityVector { get { return _remoteControl.GetNaturalGravity(); } }
        public double GravityModule { get { return GravityVector.Length(); } }

        public RemoteControlPhysicalInfo(IMyRemoteControl remoteControl)
        {
            _remoteControl = remoteControl;
        }
    }
    class Autopilot
    {
        private IMyRemoteControl _remoteControl;
        public int Accuracy { get; set; }

        public RemoteControlPhysicalInfo PointPhysicalInfo { get; private set; }

        public Autopilot(IMyTerminalBlock remoteControl)
        {
            _remoteControl = remoteControl as IMyRemoteControl;
            _remoteControl.FlightMode = FlightMode.OneWay;
            PointPhysicalInfo = new RemoteControlPhysicalInfo(remoteControl as IMyRemoteControl);
        }

        public void OnUpdate()
        {
        }

        public bool IsArrived()
        {
            if (Vector3D.Distance(_remoteControl.CurrentWaypoint.Coords, PointPhysicalInfo.Position) < Accuracy)
            {
                _remoteControl.SetAutoPilotEnabled(false);
                return true;
            }
            return false;
        }

        public void Go()
        {
            _remoteControl.SetAutoPilotEnabled(true);
        }

        public void Go(Vector3D destination)
        {
            AddDestination(destination);
            Go();
        }

        public void AddDestination(Vector3D destination)
        {
            _remoteControl.ClearWaypoints();
            _remoteControl.AddWaypoint(destination, "destination");
        }
        public void Resume()
        {
            _remoteControl.SetAutoPilotEnabled(true);
        }

        public void Stop() => _remoteControl.SetAutoPilotEnabled(false);

        public void SetSpeedLimit(int speed) => _remoteControl.SpeedLimit = speed;
    }
}