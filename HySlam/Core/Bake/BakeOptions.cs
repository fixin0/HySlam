namespace HySlam.Core.Bake
{
    /// <summary>
    /// Offline multi-pass bake configuration. A bake runs the whole clip several
    /// times without opening a window, fills the gaps left by weaker passes and
    /// exports the best-scoring result.
    /// </summary>
    internal sealed class BakeOptions
    {
        /// <summary>Number of passes over the video. One pass is a plain baseline run.</summary>
        public int Passes { get; init; } = 5;

        /// <summary>Directory that receives the final map, trajectory and reports.</summary>
        public required string OutputDirectory { get; init; }

        /// <summary>
        /// Keyframe history kept in memory per pass. A long history lets structure
        /// triangulation use baselines from much earlier in the clip.
        /// </summary>
        public int MaxKeyframes { get; init; } = 512;
    }
}
