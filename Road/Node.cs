using SimTMDG.Tools;
using System;
using System.Drawing;
using System.Xml.Serialization;

namespace SimTMDG.Road
{
    [Serializable, XmlRoot("node"), XmlType("node")]
    public class Node
    {
        #region OSM Node
        /// <summary>
        /// Node ID
        /// </summary>
        private long _id;
        [XmlAttribute("id")]
        public long Id { get { return _id; } set { _id = value; } }

        private Double _lat;
        [XmlAttribute("lat")]
        public Double Lat { get { return _lat; } set { _lat = value / 1; } }

        private Double _long;
        [XmlAttribute("lon")]
        public Double Long { get { return _long; } set { _long = value / 1; } }
        #endregion

        private static int idIndex = 0;

        #region Position
        /// <summary>
        /// Node position (Vector2D)
        /// </summary>
        private Vector2 _position;

        [XmlIgnore]
        public Vector2 Position
        {
            get { return _position; }

            set { _position = value; }
        }
        #endregion

		[XmlIgnore]
        public TrafficLight tLight;


        #region Constructor
        public Node()
        {

        }

        public Node(Vector2 position)
        {
            Id = idIndex++;
            _position = position;
        }
        #endregion


        public void latLonToPos(Double minLong, Double maxLat)
        {
            Vector2 toReturn = new Vector2(
                (Math.Ceiling((_long - minLong) * 111111)),
                (Math.Ceiling((maxLat - _lat) * 111111))
            );
            this.Position = toReturn;
        }

        #region draw
        public void Draw(Graphics g)
        {
            Color penColor = Color.Gray;

            if (tLight != null)
            {
                if (tLight.trafficLightState == TrafficLight.State.GREEN)
                {
                    penColor = Color.Green;
                }
                else if (tLight.trafficLightState == TrafficLight.State.RED)
                {
                    penColor = Color.Red;
                }

                //Pen pen = new Pen(penColor, 1);
                //Rectangle rec = new Rectangle((int) this.Position.X - 1, (int) this.Position.Y - 1, 2, 2);
                //g.DrawEllipse(pen, rec);
            }

            Pen pen = new Pen(penColor, 1);
            Rectangle rec = new Rectangle((int)this.Position.X - 1, (int)this.Position.Y - 1, 2, 2);
            g.DrawEllipse(pen, rec);

        }
        #endregion
    }
}
