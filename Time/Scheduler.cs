using System;
using System.Threading;
using System.Threading.Tasks;
using Cronos;

namespace SimTMDG.Time
{
    public class Scheduler : IDisposable
    {
        private readonly CronExpression _cronExpression;
        private readonly TimeZoneInfo _timeZoneInfo;
        private readonly Func<Task> _taskToRun;
        private readonly CancellationTokenSource _cts;

        public Scheduler(string cronExpression, Func<Task> taskToRun)
        {
            _cronExpression = CronExpression.Parse(cronExpression, CronFormat.IncludeSeconds);
            _timeZoneInfo = TimeZoneInfo.Local;
            _taskToRun = taskToRun;
            _cts = new CancellationTokenSource();
        }

        public void Start()
        {
            Task.Run(() => ScheduleJobAsync(_cts.Token));
        }

        private async Task ScheduleJobAsync(CancellationToken cancellationToken)
        {
            while (!cancellationToken.IsCancellationRequested)
            {
                var next = _cronExpression.GetNextOccurrence(DateTimeOffset.Now, _timeZoneInfo);
                if (next.HasValue)
                {
                    TimeSpan delay = next.Value - DateTimeOffset.Now;
                    if (delay.TotalMilliseconds > 0)
                        await Task.Delay(delay, cancellationToken);

                    await _taskToRun();
                }
                else
                {
                    await Task.Delay(TimeSpan.FromMinutes(1), cancellationToken);
                }
            }
        }

        public void Stop()
        {
            _cts.Cancel();
        }

        public void Dispose()
        {
            _cts.Cancel();
            _cts.Dispose();
        }
    }

}
