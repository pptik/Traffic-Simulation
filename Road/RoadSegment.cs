using SimTMDG.Tools;
using SimTMDG.Vehicle;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SimTMDG.Road
{
    [Serializable]
    public class RoadSegment : ITickable
    {
        #region road properties

        #region TEMP
        public Vector2 MidCoord = new Vector2();
        #endregion

        #region ID
        /// <summary>
        /// WaySegment ID
        /// </summary>

        private static int idIndex = 0;
        private int _id = -1;

        public int Id
        {
            get { return _id; }

            set { _id = value; }
        }
        #endregion

        #region primary properties
        /// <summary>
        /// Starting node of WaySegment
        /// </summary>
        public Node startNode;

        /// <summary>
        /// Ending node of WaySegment
        /// </summary>
        public Node endNode;

        /// <summary>
        /// WaySegment length from start node to end node
        /// </summary>
        private double _length;

        public double Length
        {
            get { return _length; }

            set { _length = value; }
        }

        public List<SegmentLane> lanes = new List<SegmentLane>();

        public double laneWidth = 3.5;

        public List<RoadSegment> nextSegment = new List<RoadSegment>();
        public List<RoadSegment> prevSegment = new List<RoadSegment>();
        #endregion


        /// <summary>
        /// WaySegment target average speed
        /// </summary>
        private int _avgSpeed;

        public int AvgSpeed
        {
            get { return _avgSpeed; }

            set { _avgSpeed = value; }
        }

        /// <summary>
        /// Way highway key / properties
        /// </summary>
        private string _highway;

        public string Highway
        {
            get { return _highway; }

            set { _highway = value; }
        }

        /// <summary>
        /// Oneway / Reverse / Bi-way
        /// </summary>
        private string _oneWay;

        public string OneWay
        {
            get { return _oneWay; }

            set { _oneWay = value; }
        }

        /// <summary>
        /// Way / Road name
        /// </summary>
        private string _roadName;

        public string RoadName
        {
            get { return _roadName; }

            set { _roadName = value; }
        }

        /// <summary>
        /// Number of lanes
        /// </summary>
        private int _numLanes;
        public int NumLanes
        {
            get { return _numLanes; }

            set { _numLanes = value; }
        }

        #endregion


        #region Constructor
        public RoadSegment()
        {
            _id = idIndex++;
        }

        public RoadSegment(Node _startNode, Node _endNode, int numlanes, string highway, string oneway)
        {
            _id = idIndex++;
            OneWay = oneway;
            startNode = _startNode;
            endNode = _endNode;
            _length = Vector2.GetDistance(startNode.Position, endNode.Position);

            Vector2 difference = new Vector2();
            difference.X = endNode.Position.X - startNode.Position.X;
            difference.Y = endNode.Position.Y - startNode.Position.Y;

            //float segmentLength = (float) Vector2.GetDistance(start, end);

            MidCoord.X = (float)(difference.X * (Length / 2)) / (float)Length + (float)startNode.Position.X;
            MidCoord.Y = (float)(difference.Y * (Length / 2)) / (float)Length + (float)startNode.Position.Y;

            _highway = highway;

            // Num lanes assumptions based on: http://wiki.openstreetmap.org/wiki/Key:lanes#Assumptions
            if ((_highway == "motorway") || (_highway == "trunk"))
            {
                if ((oneway == "true") || (oneway == "-1"))
                {
                    _numLanes = 2;
                }
                else
                {
                    _numLanes = 4;
                }
            }
            else if ((_highway == "primary") || (_highway == "secondary")
               || (_highway == "tertiary") || (_highway == "residential"))
            {
                if ((oneway == "true") || (oneway == "-1"))
                {
                    _numLanes = 1;
                }
                else
                {
                    _numLanes = 2;
                }
            }
            else
            {
                _numLanes = 1;
            }

            // If numlanes provided in the tag, use it instead of assumtions
            if (numlanes != -1)
            {
                _numLanes = numlanes;
            }

            generateLanes();
        }
        #endregion



        #region tick
        public void Tick(double tickLength)
        {
            foreach (SegmentLane lane in lanes)
            {
                lane.Tick(tickLength);
            }
        }
        #endregion




        #region draw
        public void Draw(Graphics g)
        {
            Pen pen;

            //if (OneWay == "yes")
            //{
            //    pen = new Pen(Color.Red, 1);
            //} else if (OneWay == "-1")
            //{
            //    pen = new Pen(Color.Aquamarine, 1);
            //}else
            //{
            //    pen = new Pen(Color.LightGreen, 1);
            //}

            if ((startNode.Id == 0) || (endNode.Id == 0))
            {
                pen = new Pen(Color.Red, 1);
                //Font debugFont = new Font("Calibri", 6);
                //Brush blackBrush = new SolidBrush(Color.Black);
                g.DrawLine(pen, (Point)startNode.Position, (Point)endNode.Position);
                //g.DrawString(Id.ToString(), debugFont, blackBrush, MidCoord);   //Menampilkan Teks
            }

            //g.DrawLine(pen, (Point)startNode.Position, (Point)endNode.Position);

            foreach (SegmentLane lane in lanes)
            {
                lane.Draw(g);
            }
            //if ((Id == 2917) || (Id == 25165) || (Id == 22259))
            //{
              //  Font debugFont = new Font("Calibri", 6);
              //  Brush blackBrush = new SolidBrush(Color.Black);
              //  g.DrawString(Id.ToString(), debugFont, blackBrush, MidCoord);
            //}
  
            Font debugFont1 = new Font("Calibri", 2);
            //Brush blackBrush = new SolidBrush(Color.Black);
            Brush blueBrush = new SolidBrush(Color.Blue);
            Brush redBrush = new SolidBrush(Color.Red);
            if (Id % 2==0)
            {   g.DrawString(Id.ToString(), debugFont1, blueBrush, MidCoord); }
            else
            {   g.DrawString(Id.ToString(), debugFont1, redBrush, MidCoord); }
            
        }
        #endregion




        #region helper
        public void Reset()
        {
            foreach (SegmentLane lane in lanes)
            {
                lane.Reset();
            }
        }

        void generateLanes()
        {
            Boolean forward = true;

            #region old approach
            //for (int i = 0; i < NumLanes; i++)
            //{
            //    if (NumLanes % 2 == 0) // Segment has even lanes (e.g. Numlanes= 2, 4, 6, ..)
            //    {
            //        if (i % 2 == 0) // at even lane  (e.g. at lane 2)
            //        {
            //            lanes.Add(generateSegmentLane(((i + 1) * laneWidth / 2), i, forward));
            //        }
            //        else // at odd lane  (e.g. at lane 1)
            //        {
            //            if (OneWay == "yes")
            //            {
            //                forward = true;
            //            }
            //            else if (OneWay == "-1")
            //            {
            //                forward = true;
            //            }
            //            else
            //            {
            //                forward = false;
            //            }

            //            lanes.Add(generateSegmentLane(-1 * i * laneWidth / 2, i, forward));
            //        }
            //    }
            //    else // Segment has odd lanes  (e.g. Numlanes= 1, 3, 5, ..)
            //    {
            //        if (i % 2 == 0) // at even lane
            //        {
            //            lanes.Add(generateSegmentLane((-1 * i * laneWidth / 2), i, forward));
            //        }
            //        else // at odd lane
            //        {
            //            if (OneWay == "yes")
            //            {
            //                forward = true;
            //            }
            //            else if (OneWay == "-1")
            //            {
            //                forward = true;
            //            }
            //            else
            //            {
            //                forward = false;
            //            }
            //            lanes.Add(generateSegmentLane((i * laneWidth + laneWidth) / 2, i, forward));
            //        }
            //    }
            //}
            #endregion

            #region new approach
            double maxShift = (double)(NumLanes - 1) / 2 * laneWidth;
            for (int i = 0; i < NumLanes; i++)
            {
                double shift = i * laneWidth - maxShift;
                lanes.Add(generateSegmentLane(shift, i, true));
            }
            #endregion
        }

        SegmentLane generateSegmentLane(double distance, int i, Boolean forward)
        {
            double angle = (Math.PI / 2) - Vector2.AngleBetween(this.startNode.Position, this.endNode.Position);

            Vector2 shift = new Vector2(distance * Math.Cos(angle), distance * Math.Sin(angle));
            Node newStart = new Node(new Vector2(this.startNode.Position.X + shift.X, this.startNode.Position.Y - shift.Y));
            Node newEnd = new Node(new Vector2(this.endNode.Position.X + shift.X, this.endNode.Position.Y - shift.Y));

            SegmentLane toReturn;

            if (forward)
            {
                toReturn = new SegmentLane(newStart, newEnd, i);
            }
            else
            {
                toReturn = new SegmentLane(newEnd, newStart, i);
            }

            return toReturn;
        }
        #endregion
    }
}
