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
        private int _tickCount = 0;
        private Autopilot _autopilot;
        private Logger _logger;
        public Program()
        {
            Runtime.UpdateFrequency = UpdateFrequency.Update10;

            var autopilotBlockName = "RemCon";
            var programmingBlockName = "Prog";

            _autopilot = new Autopilot(GridTerminalSystem.GetBlockWithName(autopilotBlockName));
            var programBlock = GridTerminalSystem.GetBlockWithName(programmingBlockName) as IMyProgrammableBlock;
            _logger = new Logger(programBlock);
            _logger.Clear();
            _logger.LogTrace($"Logger initialized");
        }
        public void Main(string args)
        {
            _tickCount++;
        }

        #region PreludeFooter
    }
}
#endregion