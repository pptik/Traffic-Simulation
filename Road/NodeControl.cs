using System;
using SimTMDG.Tools;
using System.Drawing;
using System.Collections.Generic;
using SimTMDG.Vehicle;
using System.Diagnostics;

namespace SimTMDG.Road
{
    [Serializable]
    public class NodeControl : ITickable
    {
        #region TEMP
        Rectangle mapBound;
        #endregion

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
            //_nodes.Clear();
            
            //foreach(WaySegment ws in Segments)
            //{
            //    ws.vehicles.Clear();
            //}

            //Segments.Clear();

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

            //foreach (WaySegment ws in Segments)
            //{
            //    foreach (IVehicle v in ws.vehicles)
            //    {
            //        v.newCoord();
            //    }
            //}
        }

        public void setBounds(Rectangle bounds)
        {
            mapBound = bounds;
        }


        #region draw
        public void Draw(Graphics g, int zoomLvl)
        {
            if (zoomLvl < 5)
            {
                foreach (RoadSegment ws in Segments)
                {
                    if ((IsInBound(ws.MidCoord, 0)))
                        ws.Draw(g);
                }
            }else if (zoomLvl < 8)
            {
                foreach (RoadSegment ws in Segments)
                {
                    if ((IsInBound(ws.startNode.Position, 0)) || (IsInBound(ws.endNode.Position, 0)))
                        ws.Draw(g);
                }
            }else if (zoomLvl < 10)
            {
                foreach (RoadSegment ws in Segments)
                {
                    if ((IsInBound(ws.startNode.Position, (int)mapBound.Width / 2)) || (IsInBound(ws.endNode.Position, (int)mapBound.Width / 2)))
                        ws.Draw(g);
                }
            }else
            {
                foreach (RoadSegment ws in Segments)
                {
                    if ((IsInBound(ws.startNode.Position, mapBound.Width)) || (IsInBound(ws.endNode.Position, mapBound.Width)))
                        ws.Draw(g);
                }
            }
            

            foreach (Node nd in _nodes)
            {
                if (IsInBound(nd.Position, 0))
                    nd.Draw(g);
            }

            foreach (RoadSegment ws in Segments)
            {
                //foreach (IVehicle v in ws.vehicles)
                //{
                //    //if (v.distance <= v.CurrentSegment.Length)
                //    if (IsInBound(v.absCoord))
                //        v.Draw(g);
                //}

                foreach (SegmentLane lane in ws.lanes)
                {
                    foreach (IVehicle v in lane.vehicles)
                    {
                        if (IsInBound(v.absCoord, 0))
                            v.Draw(g);
                    }
                }
            }

            //Pen pen = new Pen(Color.OrangeRed, 1);

            //g.DrawRectangle(pen, (float)minBound.X, (float)minBound.Y, (float)maxBound.X, (float)maxBound.Y);
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



//using SimTMDG.Time;
//using SimTMDG.Tools;
//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using System.Threading.Tasks;

//namespace SimTMDG.Road
//{
//    public class NodeControl: ITickable
//    {
//        #region Variablen und Felder

//        /// <summary>
//        /// alle verwendenten LineNodes
//        /// </summary>
//        private List<LineNode> _nodes = new List<LineNode>();
//        /// <summary>
//        /// alle verwendenten LineNodes
//        /// </summary>
//        public List<LineNode> nodes
//        {
//            get { return _nodes; }
//            set { _nodes = value; }
//        }

//        /// <summary>
//        /// alle verwendeten NodeConnections
//        /// </summary>
//        private List<NodeConnection> _connections = new List<NodeConnection>();
//        /// <summary>
//        /// alle verwendeten NodeConnections
//        /// </summary>
//        public List<NodeConnection> connections
//        {
//            get { return _connections; }
//            set { _connections = value; }
//        }

//        ///// <summary>
//        ///// Liste aller bekannten Intersections
//        ///// </summary>
//        //private List<Intersection> _intersections = new List<Intersection>();
//        ///// <summary>
//        ///// Liste aller bekannten Intersections
//        ///// </summary>
//        //public List<Intersection> intersections
//        //{
//        //    get { return _intersections; }
//        //    set { _intersections = value; }
//        //}

//        /// <summary>
//        /// List of all known network layers
//        /// </summary>
//        public List<NetworkLayer> _networkLayers { get; private set; }

//        /// <summary>
//        /// Titel dieses Layouts
//        /// </summary>
//        private string m_title;
//        /// <summary>
//        /// Titel dieses Layouts
//        /// </summary>
//        public string title
//        {
//            get { return m_title; }
//            set { m_title = value; }
//        }


//        /// <summary>
//        /// Informationstext zum Layout
//        /// </summary>
//        private string _infoText;
//        /// <summary>
//        /// Informationstext zum Layout
//        /// </summary>
//        public string infoText
//        {
//            get { return _infoText; }
//            set { _infoText = value; }
//        }


//        #endregion

//        #region Konstruktoren

//        /// <summary>
//        /// leerer Standardkonstruktor
//        /// </summary>
//        public NodeControl()
//        {
//            _networkLayers = new List<NetworkLayer>();
//        }

//        #endregion


//        #region ITickable Member

//        /// <summary>
//        /// sagt allen verwalteten Objekten Bescheid, dass sie ticken dürfen *g*
//        /// </summary>
//        public void Tick(double tickLength)
//        {
//            foreach (LineNode ln in nodes)
//            {
//                ln.Tick(tickLength);
//            }

//            int bucketNumber = GlobalTime.Instance.currentCycleTick;
//            foreach (NodeConnection nc in _connections)
//            {
//                nc.GatherStatistics(bucketNumber);
//            }
//        }

//        /// <summary>
//        /// setzt den Tick-Zustand aller LineNodes zurück
//        /// </summary>
//        public void Reset()
//        {
//            foreach (LineNode ln in nodes)
//            {
//                ln.Reset();
//            }
//        }

//        #endregion


//        public void tempLoad()
//        {
//            Vector2 position = new Vector2();
//            Vector2 inSlope = new Vector2();
//            Vector2 outSlope = new Vector2();
//            NetworkLayer networkLayer = new NetworkLayer("Layer 1", true);
//            bool stopSign = false;


//            position.X = 268;
//            position.Y = 148;
//            inSlope.X = 0;
//            inSlope.Y = 0;
//            outSlope.X = 0;
//            outSlope.Y = 0;

//            nodes.Add(new LineNode(position, inSlope, outSlope, networkLayer, stopSign));


//            position.X = 464;
//            position.Y = 180;
//            inSlope.X = -56;
//            inSlope.Y = -96;
//            outSlope.X = 56;
//            outSlope.Y = 96;

//            nodes.Add(new LineNode(position, inSlope, outSlope, networkLayer, stopSign));


//            position.X = 634;
//            position.Y = 490;
//            inSlope.X = -156;
//            inSlope.Y = -90;
//            outSlope.X = 156;
//            outSlope.Y = 90;

//            nodes.Add(new LineNode(position, inSlope, outSlope, networkLayer, stopSign));


//            position.X = 942;
//            position.Y = 282;
//            inSlope.X = -29;
//            inSlope.Y = 4;
//            outSlope.X = 29;
//            outSlope.Y = -4;

//            nodes.Add(new LineNode(position, inSlope, outSlope, networkLayer, stopSign));


//            position.X = 1044;
//            position.Y = 454;
//            inSlope.X = -128;
//            inSlope.Y = 70;
//            outSlope.X = 128;
//            outSlope.Y = -70;

//            nodes.Add(new LineNode(position, inSlope, outSlope, networkLayer, stopSign));



//            connections.Add(nodes[0], );


//        }
//    }
//}
