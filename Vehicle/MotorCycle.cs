using SimTMDG.Road;
using SimTMDG.Tools;
using System.Collections.Generic;
using System.Drawing;

namespace SimTMDG.Vehicle
{
    class MotorCycle : IVehicle
    {
        public MotorCycle(RoadSegment cs, int laneIndex, List<RoadSegment> r, long stopSegmentId)
        {
            _state.currentSegment = cs;
            _state.laneIdx = laneIndex;

            a *= (GlobalRandom.Instance.NextDouble() + 0.5);
            b *= (GlobalRandom.Instance.NextDouble() + 0.5);
            s0 *= (GlobalRandom.Instance.NextDouble() + 0.5);
            T *= (GlobalRandom.Instance.NextDouble() + 0.5);

            length = GlobalRandom.Instance.Next(2, 4);

            color = Color.FromArgb(17, 34, 78);

            Routing = new Routing();
            for (int i = 0; i < r.Count; i++)
            {
                Routing.Push(r[i]);
            }

            _physics = new IVehicle.Physics(12, 12, 0);

            this.StopSegmentId = stopSegmentId;
            this.IsQueued = false;
            this.IsReleased = false;

            newCoord();
            RotateVehicle(currentSegment.startNode, currentSegment.endNode);
        }
    }
}
