using Sandbox.ModAPI.Ingame;

namespace SpaceEngineers.CommonLibs {
    class Logger
        {
            IMyTerminalBlock _logBlock;
            public Logger(IMyTerminalBlock logBlock)
            {
                _logBlock = logBlock;
                //_logBlock.CustomData = "";
            }

            public void LogTrace(string message)
            {
                _logBlock.CustomData = _logBlock.CustomData + $"Trace: {message} \n";
            }
            public void Clear()
            {
                _logBlock.CustomData = "";
            }

            public void LogWarning(string message)
            {
                _logBlock.CustomData = _logBlock.CustomData + $"Warning: {message} \n";
            }

            public void LogError(string message, string errorMessage)
            {
                _logBlock.CustomData = _logBlock.CustomData + $"Error: {message}. Details: {errorMessage} \n";
            }
        }
}