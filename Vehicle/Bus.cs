using SimTMDG.Road;
using SimTMDG.Tools;
using System.Collections.Generic;
using System.Drawing;

namespace SimTMDG.Vehicle
{
    class Bus : IVehicle
    {
        public Bus(RoadSegment cs, int laneIndex, List<RoadSegment> r)
        {
            a = 0.9;
            b = 1.0;

            _physics = physics;
            _state.currentSegment = cs;
            _state.laneIdx = laneIndex;

            length = (GlobalRandom.Instance.Next(2) == 0) ? 12 : 14;

            color = Color.FromArgb(59, 151, 151);

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
