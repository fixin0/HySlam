using System.Diagnostics;
using System.Globalization;

namespace HySlam.Core.Bake
{
    /// <summary>
    /// Single-line percentage progress bar for the offline bake.
    ///
    /// When the output is a terminal the bar is redrawn in place with a carriage
    /// return and an ETA. When the output is redirected the bar degrades to one
    /// plain line every five percent so redirected logs stay readable.
    /// </summary>
    internal sealed class ProgressBar
    {
        private const int BarWidth = 24;
        private const int LogStepPercent = 5;

        private readonly long total;
        private readonly bool interactive;
        private readonly Stopwatch clock = Stopwatch.StartNew();

        private int lastRenderedLength;
        private int lastLoggedPercent = -100;
        private bool lineOpen;

        public ProgressBar(long totalUnits)
        {
            total = Math.Max(1, totalUnits);
            interactive = !Console.IsOutputRedirected;
        }

        /// <summary>Redraws the bar for the completed amount of work.</summary>
        public void Report(long completed, string detail)
        {
            completed = Math.Clamp(completed, 0, total);
            int percent = (int)Math.Round(completed * 100.0 / total);

            if (!interactive)
            {
                bool shouldLog = percent < 100
                    ? percent >= lastLoggedPercent + LogStepPercent
                    : lastLoggedPercent < 100;

                if (shouldLog)
                {
                    lastLoggedPercent = percent < 100 ? percent - percent % LogStepPercent : 100;
                    Console.Out.WriteLine(FormatLine(percent, detail, includeEta: true));
                    Console.Out.Flush();
                }

                return;
            }

            string line = FormatLine(percent, detail, includeEta: true);
            int padding = Math.Max(0, lastRenderedLength - line.Length);

            Console.Out.Write('\r');
            Console.Out.Write(line);
            if (padding > 0)
                Console.Out.Write(new string(' ', padding));
            Console.Out.Flush();

            lastRenderedLength = line.Length;
            lineOpen = true;
        }

        /// <summary>
        /// Terminates the bar line before a normal log message is written so the log
        /// text does not overlap the progress line.
        /// </summary>
        public void BeforeLog()
        {
            if (!interactive || !lineOpen)
                return;

            Console.Out.WriteLine();
            Console.Out.Flush();
            lineOpen = false;
            lastRenderedLength = 0;
        }

        /// <summary>Ends the bar without changing the last rendered line.</summary>
        public void Finish()
        {
            BeforeLog();
        }

        private string FormatLine(int percent, string detail, bool includeEta)
        {
            int filled = Math.Clamp((int)Math.Round(percent / 100.0 * BarWidth), 0, BarWidth);
            string bar = new string('#', filled) + new string('-', BarWidth - filled);
            string eta = includeEta ? $"  ETA {FormatEta(percent)}" : string.Empty;
            return $"Bake  [{bar}] {percent,3}%  {detail}{eta}";
        }

        private string FormatEta(int percent)
        {
            if (percent <= 0)
                return "--:--";

            double elapsed = clock.Elapsed.TotalSeconds;
            double remaining = elapsed * (100.0 - percent) / percent;
            if (!double.IsFinite(remaining) || remaining < 0.0)
                return "--:--";

            TimeSpan span = TimeSpan.FromSeconds(remaining);
            return span.TotalHours >= 1.0
                ? span.ToString(@"h\:mm\:ss", CultureInfo.InvariantCulture)
                : span.ToString(@"mm\:ss", CultureInfo.InvariantCulture);
        }
    }
}
