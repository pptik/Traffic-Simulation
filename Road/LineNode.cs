//using SimTMDG.Tools;
//using System;
//using System.Collections.Generic;
//using System.Drawing;
//using System.Linq;
//using System.Text;
//using System.Threading.Tasks;
//using System.Xml.Serialization;

//namespace SimTMDG.Road
//{
//    /// <summary>
//    /// Junction between two Node Connections
//    /// </summary>
//    [Serializable]
//    public class LineNode : ITickable
//    {
//        #region Variables

//        #region Static fields for drawing
//        private static Pen blackPen = new Pen(Color.Black);
//        private static Brush blackBrush = new SolidBrush(Color.Black);
//        private static Brush greenBrush = new SolidBrush(Color.Green);
//        private static Brush redBrush = new SolidBrush(Color.Red);

//        private static PointF[] _stopSignEdgeOffsets = null;
//        private static void InitStopSignEdgeOffsets()
//        {
//            if (_stopSignEdgeOffsets == null)
//            {
//                _stopSignEdgeOffsets = new PointF[8];
//                for (int i = 0; i < 8; ++i)
//                {
//                    _stopSignEdgeOffsets[i] = new PointF((float)(12 * Math.Sin((2 * i + 1) * Math.PI / 8.0)), (float)(12 * Math.Cos((2 * i + 1) * Math.PI / 8.0)));
//                }
//            }
//        }
//        #endregion

//        #region Flag
//        /// <summary>
//        /// for associated traffic light LineNode
//        /// </summary>
//        //[XmlIgnore]
//        //public TrafficLight tLight;

//        /// <summary>
//        /// Flag whether this LineNode models a stop sign
//        /// </summary>
//        private bool _stopSign;
//        /// <summary>
//        /// Flag whether this LineNode models a stop sign
//        /// </summary>
//        public bool stopSign
//        {
//            get { return _stopSign; }
//            set { _stopSign = value; }
//        }
//        #endregion

//        #region Position(s) of node(s)
//        /// <summary>
//        /// absolute position of the Line Nodes in world coordinates
//        /// </summary>
//        private Vector2 _position;
//        /// <summary>
//        /// absolute position of the Line Nodes in world coordinates
//        /// </summary>
//        public Vector2 position
//        {
//            get { return _position; }
//            set { _position = value; UpdateNodeConnections(true); UpdateNodeGraphics(); }//}
//        }
//        /// <summary>
//        /// Square of side length 8 to the absolute position of the Line Nodes in world coordinates
//        /// </summary>
//        public RectangleF positionRect
//        {
//            get { return new RectangleF((float)position.X - 6, (float)position.Y - 6, 12, 12); }
//        }


//        /// <summary>
//        /// Einflugvektor
//        /// </summary>
//        private Vector2 _in;
//        /// <summary>
//        /// Ausflugvektor
//        /// </summary>
//        private Vector2 _out;
//        /// <summary>
//        /// relativer Einflugvektor
//        /// </summary>
//        public Vector2 inSlope
//        {
//            get { return _in; }
//            set { _in = value; UpdateNodeConnections(true); UpdateNodeGraphics(); } //}
//        }
//        /// <summary>
//        /// absolute position of the test flight vector anchor in world coordinates
//        /// </summary>
//        //[XmlIgnore]
//        public Vector2 inSlopeAbs
//        {
//            get { return position + _in; }
//            set { _in = value - position; UpdateNodeConnections(true); UpdateNodeGraphics(); }// }
//        }
//        /// <summary>
//        /// Square of side length 8 to the absolute position of the test flight vector anchor in world coordinates
//        /// </summary>
//        public RectangleF inSlopeRect
//        {
//            get { return new RectangleF((float)inSlopeAbs.X - 4, (float)inSlopeAbs.Y - 4, 8, 8); }
//        }

//        /// <summary>
//        /// relativer Ausflugvektor
//        /// </summary>
//        public Vector2 outSlope
//        {
//            get { return _out; }
//            set { _out = value; UpdateNodeConnections(true); UpdateNodeGraphics(); } // }
//        }
//        /// <summary>
//        /// Square of side length 8 to the absolute position of the trip armature vector in world coordinates
//        /// </summary>
//        public RectangleF outSlopeRect
//        {
//            get { return new RectangleF((float)outSlopeAbs.X - 4, (float)outSlopeAbs.Y - 4, 8, 8); }
//        }
//        /// <summary>
//        /// absolute position of the trip armature vector in world coordinates
//        /// </summary>
//        //[XmlIgnore]
//        public Vector2 outSlopeAbs
//        {
//            get { return position + _out; }
//            set { _out = value - position; UpdateNodeConnections(true); UpdateNodeGraphics(); } // }
//        }

//        #endregion

//        #region Hashcodes

//        /*
//		 * Nachdem der ursprüngliche Ansatz zu Hashen zu argen Kollisionen geführt hat, nun eine verlässliche Methode für Kollisionsfreie Hashes 
//		 * mittels eindeutiger IDs für jeden LineNode die über statisch Klassenvariablen vergeben werden
//		 */

//        /// <summary>
//        /// Klassenvariable welche den letzten vergebenen hashcode speichert und bei jeder Instanziierung eines Objektes inkrementiert werden muss
//        /// </summary>
//        private static int hashcodeIndex = 0;

//        /// <summary>
//        /// Hashcode des instanziierten Objektes
//        /// </summary>
//        public int hashcode = -1;

//        /// <summary>
//        /// gibt den Hashcode des Fahrzeuges zurück.
//        /// </summary>
//        /// <returns></returns>
//        public override int GetHashCode()
//        {
//            return hashcode;
//        }

//        /// <summary>
//        /// Setzt die statische Klassenvariable hashcodeIndex zurück. Achtung: darf nur in bestimmten Fällen aufgerufen werden.
//        /// </summary>
//        public static void ResetHashcodeIndex()
//        {
//            hashcodeIndex = 0;
//        }

//        #endregion

//        #region NetworkLayer stuff

//        /// <summary>
//        /// NetworkLayer this LineNode lies on.
//        /// </summary>
//        private NetworkLayer _networkLayer;
//        /// <summary>
//        /// NetworkLayer this LineNode lies on.
//        /// </summary>
//        [XmlIgnore]
//        public NetworkLayer networkLayer
//        {
//            get { return _networkLayer; }
//            set { _networkLayer = value; }
//        }

//        /// <summary>
//        /// Returns whether this LineNode is visible (this is the case if _networkLayer is either null or set to visible).
//        /// </summary>
//        public bool isVisible { get { return networkLayer == null || networkLayer.visible; } }

//        #endregion

//        #endregion


//        #region constructor
//        /// <summary>
//        /// Standardkonstruktor, wird nur für die XML Serialisierung benötigt.
//        /// NICHT VERWENDEN, dieser Konstruktor vergibt keine neuen Hashcodes!
//        /// </summary>
//        public LineNode()
//        {
//            this.position = new Vector2(0, 0);
//            this.inSlope = new Vector2(0, 0);
//            this.outSlope = new Vector2(0, 0);

//            hashcode = hashcodeIndex++;
//            InitStopSignEdgeOffsets();
//        }

//        /// <summary>
//        /// Konstruktor, erstell einen neuen LineNode an der Position position.
//        /// in-/outSlope werden mit (0,0) initialisiert.
//        /// </summary>
//        /// <param name="position">absolute Position</param>
//        /// <param name="nl">Network layer of this LineNode</param>
//        /// <param name="stopSign">Flag whether this LineNode models a stop sign</param>
//        public LineNode(Vector2 position, NetworkLayer nl, bool stopSign)
//        {
//            this.position = position;
//            this.inSlope = new Vector2(0, 0);
//            this.outSlope = new Vector2(0, 0);
//            _networkLayer = nl;
//            _stopSign = stopSign;

//            // Hashcode vergeben
//            hashcode = hashcodeIndex++;
//            InitStopSignEdgeOffsets();
//        }

//        /// <summary>
//        /// Konstruktor, erstell einen neuen LineNode an der Position position.
//        /// </summary>
//        /// <param name="Position">absolute Position</param>
//        /// <param name="inSlope">eingehender Richtungsvektor</param>
//        /// <param name="outSlope">ausgehender Richtungsvektor</param>
//        /// <param name="nl">Network layer of this LineNode</param>
//        /// <param name="stopSign">Flag whether this LineNode models a stop sign</param>
//        public LineNode(Vector2 Position, Vector2 inSlope, Vector2 outSlope, NetworkLayer nl, bool stopSign)
//        {
//            this.position = Position;
//            this.inSlope = inSlope;
//            this.outSlope = outSlope;
//            _networkLayer = nl;
//            _stopSign = stopSign;

//            // Hashcode vergeben
//            hashcode = hashcodeIndex++;
//            InitStopSignEdgeOffsets();
//        }
//        #endregion

//        #region Connections

//        /// <summary>
//        /// vorherige NodeConnections
//        /// </summary>
//        private List<NodeConnection> _nextConnections = new List<NodeConnection>(); // Vorheriger Knoten
//                                                                                    /// <summary>
//                                                                                    /// nachfolgende NodeConnections
//                                                                                    /// </summary>
//        private List<NodeConnection> _prevConnections = new List<NodeConnection>(); // Nachfolgender Knoten

//        /// <summary>
//        /// nachfolgende NodeConnections
//        /// </summary>
//        [XmlIgnore]
//        public List<NodeConnection> nextConnections
//        {
//            get { return _nextConnections; }
//        }
//        /// <summary>
//        /// vorherige NodeConnections
//        /// </summary>
//        [XmlIgnore]
//        public List<NodeConnection> prevConnections
//        {
//            get { return _prevConnections; }
//        }


//        /// <summary>
//        /// sucht  die NodeConnection zum LineNode lineNode heraus
//        /// </summary>
//        /// <param name="lineNode">zu suchender LineNode</param>
//        /// <returns>erstbeste NodeConnection in nextConnections mit (nc.endNode == lineNode) oder null</returns>
//        public NodeConnection GetNodeConnectionTo(LineNode lineNode)
//        {
//            foreach (NodeConnection lc in _nextConnections)
//            {
//                if (lc.endNode == lineNode)
//                {
//                    return lc;
//                }
//            }
//            return null;
//        }

//        /// <summary>
//        /// Aktualisiert sämtliche NodeConnections in nextNodes
//        /// </summary>
//		/// <param name="doRecursive">soll die Aktualisierung auch bei den prevNodes durchgeführt werden</param>
//        public void UpdateNodeConnections(bool doRecursive)
//        {
//            foreach (NodeConnection lc in _nextConnections)
//            {
//                lc.lineSegment = null;
//                lc.lineSegment = new LineSegment(0, this.position, this.outSlopeAbs, lc.endNode.inSlopeAbs, lc.endNode.position);
//            }
//            if (doRecursive)
//            {
//                foreach (NodeConnection lc in _prevConnections)
//                {
//                    lc.startNode.UpdateNodeConnections(false);
//                }
//            }
//        }
//        #endregion


//        #region Graphics
//        /// <summary>
//        /// ein GraphicsPath, der die wichtigen Grafiken des Knotens Enthält (Ankerpunkte etc)
//        /// </summary>
//        private System.Drawing.Drawing2D.GraphicsPath[] _nodeGraphics = new System.Drawing.Drawing2D.GraphicsPath[4];
//        /// <summary>
//        /// ein GraphicsPath, der die wichtigen Grafiken des Knotens Enthält (Ankerpunkte etc)
//        /// </summary>
//        public System.Drawing.Drawing2D.GraphicsPath[] nodeGraphics
//        {
//            get { return _nodeGraphics; }
//        }

//        /// <summary>
//        /// aktualisiert das Feld nodeGraphics
//        /// </summary>
//        private void UpdateNodeGraphics()
//        {
//            if ((position != null) && (inSlope != null) && (outSlope != null))
//            {
//                // Linien zu den Stützpunkten
//                _nodeGraphics[0] = new System.Drawing.Drawing2D.GraphicsPath(new PointF[] { inSlopeAbs, position }, new byte[] { 1, 1 });
//                _nodeGraphics[1] = new System.Drawing.Drawing2D.GraphicsPath(new PointF[] { outSlopeAbs, position }, new byte[] { 1, 1 });

//                // Stützpunkte
//                System.Drawing.Drawing2D.GraphicsPath inPoint = new System.Drawing.Drawing2D.GraphicsPath();
//                inPoint.AddEllipse(inSlopeRect);
//                _nodeGraphics[2] = inPoint;

//                System.Drawing.Drawing2D.GraphicsPath outPoint = new System.Drawing.Drawing2D.GraphicsPath();
//                // wir versuchen ein Dreieck zu zeichnen *lol*
//                Vector2 dir = outSlope.Normalized;
//                outPoint.AddPolygon(
//                    new PointF[] {
//                        (6*dir) + outSlopeAbs,
//                        (6*dir.RotateCounterClockwise(Math.PI * 2 / 3)) + outSlopeAbs,
//                        (6*dir.RotateCounterClockwise(Math.PI * 4 / 3)) + outSlopeAbs,
//                        (6*dir) + outSlopeAbs
//                    });
//                _nodeGraphics[3] = outPoint;
//            }
//        }

//        #endregion


//        #region ITickable Member
//        public void Tick(double tickLength)
//        {
//            throw new NotImplementedException();
//        }

//        public void Reset()
//        {
//            throw new NotImplementedException();
//        }
//        #endregion
//    }

//}
