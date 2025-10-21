using SimTMDG.Road;
using SimTMDG.Tools;
using System.Collections.Generic;
using System.Drawing;

namespace SimTMDG.Vehicle
{
    class Truck : IVehicle
    {
        public Truck(RoadSegment cs, int laneIndex, List<RoadSegment> r)
        {
            a = 0.6;
            b = 0.8;

            _physics = physics;
            _state.currentSegment = cs;
            _state.laneIdx = laneIndex;

            a *= (GlobalRandom.Instance.NextDouble() + 0.5);
            b *= (GlobalRandom.Instance.NextDouble() + 0.5);
            s0 *= (GlobalRandom.Instance.NextDouble() + 0.5);
            T *= (GlobalRandom.Instance.NextDouble() + 0.5);

            length = (GlobalRandom.Instance.Next(2) == 0) ? 10 : 12;

            color = Color.FromArgb(191, 9, 47);

            Routing = new Routing();
            for (int i = 0; i < r.Count; i++)
            {
                Routing.Push(r[i]);
            }
            _physics = new IVehicle.Physics(14, 14, 0);

            newCoord();
            RotateVehicle(currentSegment.startNode, currentSegment.endNode);
        }
    }
}
