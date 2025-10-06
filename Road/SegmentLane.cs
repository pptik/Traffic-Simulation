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
    public class SegmentLane : ITickable
    {
        #region lane properties
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

        public int LaneIdx = 0;
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

        #region constructor
        public SegmentLane(Node _startNode, Node _endNode, int laneIdx)
        {
            _id = idIndex++;
            LaneIdx = laneIdx;
            startNode = _startNode;
            endNode = _endNode;
            _length = Vector2.GetDistance(startNode.Position, endNode.Position);

            Vector2 difference = new Vector2();
            difference.X = endNode.Position.X - startNode.Position.X;
            difference.Y = endNode.Position.Y - startNode.Position.Y;
        }
        #endregion


        #region tick
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

            //RemoveAllVehiclesInRemoveList();
        }


        #region remove vehicle
        public void RemoveAllVehiclesInRemoveList()
        {
            foreach (IVehicle v in vehToRemove)
            {
                vehicles.Remove(v);
                v.Reset();
            }

            //vehToRemove.Clear();
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

        #endregion


        public void Reset()
        {
            foreach (IVehicle v in vehicles)
            {
                v.Reset();
            }
        }


        #region tools
        public int findVehicleInFront(double position)
        {
            //// Binary Search
            //int low = 0;
            //int high = vehicles.Count - 1;

            //while (low <= high)
            //{
            //    int mid = (low + high) >> 1;
            //    double rearPos = vehicles[mid].distance;
            //    // final int compare = Double.compare(midPos, vehiclePos);
            //    // note vehicles are sorted in reverse order of position
            //    if (position > rearPos)
            //    {
            //        low = mid + 1;
            //    }
            //    else if (position < rearPos)
            //    {
            //        high = mid - 1;
            //    }
            //    else
            //    {
            //        return mid; // key found
            //    }
            //}
            //return -(low + 1); // key not found

            // naive search
            if (vehicles.Count < 1)
            {
                return -1;
            }

            for (int i = 0; i < vehicles.Count; i++)
            {
                if (vehicles[i].RearPos > position)
                {
                    return i;
                }
            }

            return -1;
        }

        public int findVehicleBehind(double position)
        {
            if (vehicles.Count < 1)
            {
                return -1;
            }

            int toReturn = -1;

            for (int i = 0; i < vehicles.Count; i++)
            {
                if (vehicles[i].distance < position)
                {
                    toReturn = i;
                }
            }

            return toReturn;
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
        #endregion
    }
}
