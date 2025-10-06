using SimTMDG.Tools;
using SimTMDG.Vehicle;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SimTMDG.Road
{
    [Serializable]
    public class WaySegment : ITickable
    {
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

        #region road properties

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

        #region vehicles on segment
        public List<IVehicle> vehicles = new List<IVehicle>();
        public List<IVehicle> vehToRemove = new List<IVehicle>();
        #endregion


        #region Constructor
        public WaySegment()
        {
            _id = idIndex++;
        }


        public WaySegment(Node _startNode, Node _endNode)
        {
            _id = idIndex++;
            startNode = _startNode;
            endNode = _endNode;
            _length = Vector2.GetDistance(startNode.Position, endNode.Position);

            Vector2 difference = new Vector2();
            difference.X = endNode.Position.X - startNode.Position.X;
            difference.Y = endNode.Position.Y - startNode.Position.Y;

            //float segmentLength = (float) Vector2.GetDistance(start, end);

            MidCoord.X = (float)(difference.X * (Length / 2)) / (float) Length + (float) startNode.Position.X;
            MidCoord.Y = (float)(difference.Y * (Length / 2)) / (float) Length + (float) startNode.Position.Y;
        }

        public WaySegment(Node _startNode, Node _endNode, int numlanes, string highway, string oneway)
        {
            _id = idIndex++;
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
                if ((oneway == "true")||(oneway == "-1"))
                {
                    _numLanes = 2;
                }else
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
            else {
                _numLanes = 1;
            }

            // If numlanes provided in the tag, use it instead of assumtions
            if (numlanes != -1)
            {
                _numLanes = numlanes;
            }
        }


        #endregion


        #region draw
        public void Draw(Graphics g)
        {
            Pen pen = new Pen(Color.DarkGray, 1);
            g.DrawLine(pen, (Point)startNode.Position, (Point)endNode.Position);

            //Font debugFont = new Font("Calibri", 6);
            //Brush blackBrush = new SolidBrush(Color.Black);
            //g.DrawString(Id.ToString(), debugFont, blackBrush, MidCoord);
        }

        public void Tick(double tickLength)
        {
            SortVehicles();

            for (int i = 0; i < vehicles.Count; i++)
            {
                vehicles[i].Think(tickLength);
            }

            RemoveAllVehiclesInRemoveList();

            for (int i = 0; i < vehicles.Count; i++)
            {
                vehicles[i].Move(tickLength);
            }

            RemoveAllVehiclesInRemoveList();

            
        }

        public void RemoveAllVehiclesInRemoveList()
        {
            foreach (IVehicle v in vehToRemove)
            {
                vehicles.Remove(v);
            }

            vehToRemove.Clear();
        }

        public void Reset()
        {
            foreach (IVehicle v in vehicles)
            {
                v.Reset();
            }
        }
        #endregion


        #region sort vehicles
        public void SortVehicles()
        {
            if (vehicles.Count > 1)
            {
                vehicles.Sort((x, y) => x.distance.CompareTo(y.distance));

                for (int i = 0; i < vehicles.Count; i++)
                {
                    vehicles[i].vehiclesIndex = i;
                }
            }
        }
        #endregion
    }
}
