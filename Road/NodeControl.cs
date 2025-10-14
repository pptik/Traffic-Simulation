using System;
using SimTMDG.Tools;
using System.Drawing;
using System.Collections.Generic;
using SimTMDG.Vehicle;

namespace SimTMDG.Road
{
    [Serializable]
    public class NodeControl : ITickable
    {
        #region TEMP
        Rectangle mapBound;
        #endregion

        public int ActiveVehicleCount
        {
            get
            {
                int total = 0;
                foreach (RoadSegment ws in Segments)
                {
                    foreach (SegmentLane lane in ws.lanes)
                    {
                        total += lane.vehicles.Count;
                    }
                }
                return total;
            }
        }


        //private Routing _route;
        public List<Node> _nodes = new List<Node>();
        public List<RoadSegment> segments = new List<RoadSegment>();

        internal List<RoadSegment> Segments
        {
            get
            {
                return segments;
            }

            set
            {
                segments = value;
            }
        }

        public NodeControl()
        {
            
        }

        // TODO temp for testing
        public void Load()
        {

            Clear();
        }


        public void Clear()
        {
            _nodes.Clear();

            foreach (RoadSegment ws in Segments)
            {
                //ws.vehicles.Clear();

                foreach(SegmentLane lane in ws.lanes)
                {
                    lane.vehicles.Clear();
                }
            }

            Segments.Clear();
        }

        public void Reset()
        {
            foreach (RoadSegment ws in segments)
            {
                ws.Reset();
            }

        }

        public void Tick(double tickLength)
        {
            foreach(RoadSegment ws in Segments)
            {
                ws.Tick(tickLength);              
            }
        }

        public void setBounds(Rectangle bounds)
        {
            mapBound = bounds;
        }


        #region draw
        public void Draw(Graphics g, int zoomLvl)
        {
            if (Segments == null) return;

            if (zoomLvl < 5)
            {
                foreach (RoadSegment ws in Segments)
                {
                    if (ws == null || ws.MidCoord == null) continue;

                    if (IsInBound(ws.MidCoord, 0))
                        ws.Draw(g);
                }
            }
            else if (zoomLvl < 8)
            {
                foreach (RoadSegment ws in Segments)
                {
                    if (ws == null || ws.startNode == null || ws.endNode == null) continue;
                    if (ws.startNode.Position == null || ws.endNode.Position == null) continue;

                    if (IsInBound(ws.startNode.Position, 0) || IsInBound(ws.endNode.Position, 0))
                        ws.Draw(g);
                }
            }
            else if (zoomLvl < 10)
            {
                foreach (RoadSegment ws in Segments)
                {
                    if (ws == null || ws.startNode == null || ws.endNode == null) continue;
                    if (ws.startNode.Position == null || ws.endNode.Position == null) continue;

                    if (IsInBound(ws.startNode.Position, (int)mapBound.Width / 2) ||
                        IsInBound(ws.endNode.Position, (int)mapBound.Width / 2))
                        ws.Draw(g);
                }
            }
            else
            {
                foreach (RoadSegment ws in Segments)
                {
                    if (ws == null || ws.startNode == null || ws.endNode == null) continue;
                    if (ws.startNode.Position == null || ws.endNode.Position == null) continue;

                    if (IsInBound(ws.startNode.Position, mapBound.Width) ||
                        IsInBound(ws.endNode.Position, mapBound.Width))
                        ws.Draw(g);
                }
            }

            if (_nodes != null)
            {
                foreach (Node nd in _nodes)
                {
                    if (nd == null || nd.Position == null) continue;

                    if (IsInBound(nd.Position, 0))
                        nd.Draw(g);
                }
            }

            foreach (RoadSegment ws in Segments)
            {
                if (ws == null || ws.lanes == null) continue;

                foreach (SegmentLane lane in ws.lanes)
                {
                    if (lane == null || lane.vehicles == null) continue;

                    foreach (IVehicle v in lane.vehicles)
                    {
                        if (v == null || v.absCoord == null) continue;

                        if (IsInBound(v.absCoord, 0))
                            v.Draw(g);
                    }
                }
            }
        }

        #endregion

        public bool IsInBound(Vector2 coord, int extension)
        {
            if  ((coord.X < mapBound.X - extension)||(coord.Y < mapBound.Y - extension) ||
                 (coord.X > mapBound.X + mapBound.Width + extension) ||
                 (coord.Y > mapBound.Y + mapBound.Height + extension)){
                return false;
            }else{
                return true;
            }
        }
    }    
}