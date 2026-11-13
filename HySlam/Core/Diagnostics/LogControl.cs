using Serilog.Core;
using Serilog.Events;

namespace HySlam.Core.Diagnostics
{
    /// <summary>
    /// Runtime control of the Serilog minimum level. The bake lowers the level while
    /// it draws the progress bar so per-frame engine traces do not collide with it,
    /// then restores normal logging between passes.
    /// </summary>
    internal static class LogControl
    {
        public static readonly LoggingLevelSwitch LevelSwitch = new(LogEventLevel.Information);

        public static void Quiet()
        {
            LevelSwitch.MinimumLevel = LogEventLevel.Warning;
        }

        public static void Normal()
        {
            LevelSwitch.MinimumLevel = LogEventLevel.Information;
        }
    }
}
