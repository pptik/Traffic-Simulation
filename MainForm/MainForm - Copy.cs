using SimTMDG.Road;
using SimTMDG.Time;
using SimTMDG.Tools;
using SimTMDG.Vehicle;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Xml;
using System.Xml.Linq;
using System.Xml.Serialization;

namespace SimTMDG
{
    public partial class Main : Form
    {
        #region TEMP
        private double _temp_stepsPerSeconds = 10;
        private double _temp_simulationDuration = 15;
        private NodeControl nc;
        private List<RoadSegment> _route;
        private List<RoadSegment> _route2;
        //private List<RoadSegment> _route3;
        //private List<RoadSegment> _route4;
        //private List<RoadSegment> _route5;
        //private List<RoadSegment> _route6;
        Random rnd = new Random();
        int vehCount = 0;
        int activeVehicles = 0;
        Double timeMod = 0.0;
        Bitmap bmp;
        Bitmap bmpZoom;
        double minLon;
        double maxLon;
        double minLat;
        double maxLat;
        Boolean boundsDefined = false;
        Rectangle renderedRect;
        #endregion


        #region Helper

        /// <summary>
        /// stores the window state
        /// </summary>
        [Serializable]
        public struct WindowSettings
        {
            /// <summary>
            /// Window state
            /// </summary>
            public FormWindowState _windowState;
            /// <summary>
            /// Position of window
            /// </summary>
            public Point _position;
            /// <summary>
            /// Size of window
            /// </summary>
            public Size _size;
        }

        private enum DragNDrop
        {
            NONE,
            MOVE_MAIN_GRID,
            MOVE_NODES,
            CREATE_NODE,
            MOVE_IN_SLOPE, MOVE_OUT_SLOPE,
            MOVE_TIMELINE_BAR, MOVE_EVENT, MOVE_EVENT_START, MOVE_EVENT_END,
            MOVE_THUMB_RECT,
            DRAG_RUBBERBAND
        }

        /// <summary>
        /// MainForm invalidation level
        /// </summary>
        public enum InvalidationLevel
        {
            /// <summary>
            /// invalidate everything
            /// </summary>
            ALL,
            /// <summary>
            /// invalidate only main canvas
            /// </summary>
            ONLY_MAIN_CANVAS,
            /// <summary>
            /// invalidate main canvas and timeline
            /// </summary>
            MAIN_CANVAS_AND_TIMELINE
        }

        #endregion

        #region Variables / Properties
        /// <summary>
        /// Simulation playing status
        /// </summary>
        private bool simIsPlaying = false;

        /// <summary>
		/// Stopwatch for timing of rendering
		/// </summary>
		private System.Diagnostics.Stopwatch renderStopwatch = new System.Diagnostics.Stopwatch();

        /// <summary>
        /// Stopwatch for timing the traffic logic
        /// </summary>
        private System.Diagnostics.Stopwatch thinkStopwatch = new System.Diagnostics.Stopwatch();


        /// <summary>
        /// NodeSteuerung
        /// </summary>
        //public NodeControl nodeControl = new NodeControl();

        private DragNDrop howToDrag = DragNDrop.NONE;

        private Rectangle daGridRubberband;

        /// <summary>
        /// AutoscrollPosition vom daGrid umschließenden Panel. (Wird für Thumbnailanzeige benötigt)
        /// </summary>
        private Point daGridScrollPosition = new Point();

        /// <summary>
        /// Mittelpunkt der angezeigten Fläche in Weltkoordinaten. (Wird für Zoom benötigt)
        /// </summary>
        private PointF daGridViewCenter = new Point();

        //private List<GraphicsPath> additionalGraphics = new List<GraphicsPath>();

        private float[,] zoomMultipliers = new float[,] {
            { 0.05f, 20},
            { 0.1f, 10},
            { 0.15f, 1f/0.15f},
            { 0.2f, 5},
            { 0.25f, 4},
            { 1f/3f, 3},
            { 0.5f, 2},
            { 2f/3f, 1.5f},
            { 1, 1},
            { 1.5f, 2f/3f},
            { 2, 0.5f},
            { 4, 0.25f},
            { 8, 0.125f}
        };



        private int[] speedMultipliers = new int[]
        {
            1, 2, 4, 8, 16
        };

        #endregion

        public Main()
        {
            // - maxtime

            InitializeComponent();

            // - colorlist
            // - setdockingstuff

            //
            speedComboBox.SelectedIndex = 0;
            zoomComboBox.SelectedIndex = 8;
            daGridScrollPosition = new Point(0, 0);
            renderedRect = new Rectangle();
            UpdateDaGridClippingRect();
            DaGrid.Dock = DockStyle.Fill;

            // - setstyle
            this.SetStyle(ControlStyles.OptimizedDoubleBuffer | ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint, true);

            // - renderoptions


            //temp
            //roadSegment = new RoadSegment();
            nc = new NodeControl();
        }


        #region timer
        private void timerSimulation_Tick(object sender, EventArgs e)
        {
            thinkStopwatch.Reset();
            thinkStopwatch.Start();

            double tickLength = 1.0d / _temp_stepsPerSeconds; //(double)stepsPerSecondSpinEdit.Value;
            //Debug.WriteLine("timerSimulation Interval " + timerSimulation.Interval + ", ticklength: " + tickLength);


            //if (GlobalTime.Instance.currentTime < _temp_simulationDuration && (GlobalTime.Instance.currentTime + tickLength) >= _temp_simulationDuration)
            //{
            //    //cbEnableSimulation.Checked = false;

            //    // TODO playButton_click?
            //    simIsPlaying = false;
            //    timerSimulation.Enabled = simIsPlaying;
            //    playButton.Text = "Play";
            //}
            //timelineSteuerung.Advance(tickLength);
            GlobalTime.Instance.Advance(tickLength);

            //roadSegment.Tick(tickLength);
            nc.Tick(tickLength);

            nc.Reset();

            generateVehicles();

            ////tickCount++;

            //nodeSteuerung.Tick(tickLength);
            //trafficVolumeSteuerung.Tick(tickLength);

            //nodeSteuerung.Reset();

            thinkStopwatch.Stop();
            //Debug.WriteLine(GlobalTime.Instance.currentTime);
            Invalidate(InvalidationLevel.MAIN_CANVAS_AND_TIMELINE);
        }


        #endregion


        #region UI event
        private void playButton_Click(object sender, EventArgs e)
        {
            if (!simIsPlaying)
            {
                playButton.Text = "Pause";

            }
            else
            {
                playButton.Text = "Play";
            }

            simIsPlaying = !simIsPlaying;
            timerSimulation.Enabled = simIsPlaying;
        }

        private void stepButton_Click(object sender, EventArgs e)
        {
            timerSimulation_Tick(sender, e);
            Invalidate(InvalidationLevel.MAIN_CANVAS_AND_TIMELINE);
        }

        #endregion


        #region DaGrid
        void DaGrid_MouseWheel(object sender, MouseEventArgs e)
        {
            if ((Control.ModifierKeys & Keys.Control) == Keys.Control)
            {
                zoomComboBox.SelectedIndex = Math2.Clamp(zoomComboBox.SelectedIndex + (e.Delta / 120), 0, zoomComboBox.Items.Count - 1);

            }
        }


        private void DaGrid_Resize(object sender, EventArgs e)
        {
            //throw new NotImplementedException();
        }
        #endregion


        #region paint
        void DaGrid_Paint(object sender, PaintEventArgs e)
        {
            //Debug.WriteLine("DaGrid Paint");
            e.Graphics.SmoothingMode = SmoothingMode.HighQuality;
            e.Graphics.InterpolationMode = InterpolationMode.HighQualityBicubic;

            renderStopwatch.Reset();
            renderStopwatch.Start();

            //if (bmp != null)
            //{
            //    // Draw Background
            //    //e.Graphics.DrawImage(bmp, Point.Empty);
            //    Rectangle srcRect = new Rectangle(daGridScrollPosition.X, daGridScrollPosition.Y, DaGrid.Width, DaGrid.Height);
            //    DrawZoom(e.Graphics, srcRect);
            //}


            e.Graphics.Transform = new Matrix(
                zoomMultipliers[zoomComboBox.SelectedIndex, 0], 0,
                0, zoomMultipliers[zoomComboBox.SelectedIndex, 0],
                -daGridScrollPosition.X * zoomMultipliers[zoomComboBox.SelectedIndex, 0], -daGridScrollPosition.Y * zoomMultipliers[zoomComboBox.SelectedIndex, 0]);

            //roadSegment.Draw(e.Graphics);  
            //nc.Draw(e.Graphics);

            if (boundsDefined)
            {
                nc.Draw(e.Graphics, zoomComboBox.SelectedIndex);
            }

            //Pen pen = new Pen(Color.OrangeRed, 1);
            //e.Graphics.DrawRectangle(pen, renderedRect.X, renderedRect.Y, renderedRect.Width, renderedRect.Height);

            // Draw Foreground
            //foreach (WaySegment ws in nc.segments)
            //{
            //    foreach (IVehicle v in ws.vehicles)
            //    {
            //        v.Draw(e.Graphics);
            //    }
            //}



            renderStopwatch.Stop();


            e.Graphics.Transform = new Matrix(1, 0, 0, 1, 0, 0);
            e.Graphics.DrawString(
                "thinking time: " + thinkStopwatch.ElapsedMilliseconds + "ms, possible thoughts per second: " + ((thinkStopwatch.ElapsedMilliseconds != 0) ? (1000 / thinkStopwatch.ElapsedMilliseconds).ToString() : "-"),
                new Font("Arial", 10),
                new SolidBrush(Color.Black),
                8,
                40);

            e.Graphics.DrawString(
                "rendering time: " + renderStopwatch.ElapsedMilliseconds + "ms, possible fps: " + ((renderStopwatch.ElapsedMilliseconds != 0) ? (1000 / renderStopwatch.ElapsedMilliseconds).ToString() : "-"),
                new Font("Arial", 10),
                new SolidBrush(Color.Black),
                8,
                56);

            e.Graphics.DrawString(
                "Active Vehicles: " + activeVehicles,
                new Font("Arial", 10),
                new SolidBrush(Color.Black),
                8,
                72);
        }


        private void Invalidate(InvalidationLevel il)
        {
            base.Invalidate();
            switch (il)
            {
                case InvalidationLevel.ALL:
                    //thumbGrid.Invalidate();
                    DaGrid.Invalidate();
                    break;
                case InvalidationLevel.MAIN_CANVAS_AND_TIMELINE:
                    DaGrid.Invalidate();
                    break;
                case InvalidationLevel.ONLY_MAIN_CANVAS:
                    DaGrid.Invalidate();
                    break;
                default:
                    break;
            }
        }

        private void tableLayoutPanel1_Paint(object sender, PaintEventArgs e)
        {

        }

        /// <summary>
		/// aktualisiert das Clipping-Rectangle von DaGrid
		/// </summary>
		private void UpdateDaGridClippingRect()
        {
            if (zoomComboBox.SelectedIndex >= 0)
            {
                //    // daGridClippingRect aktualisieren
                //renderOptionsDaGrid.clippingRect.X = daGridScrollPosition.X;
                //renderOptionsDaGrid.clippingRect.Y = daGridScrollPosition.Y;
                //renderOptionsDaGrid.clippingRect.Width = (int)Math.Ceiling(pnlMainGrid.ClientSize.Width * zoomMultipliers[zoomComboBox.SelectedIndex, 1]);
                //renderOptionsDaGrid.clippingRect.Height = (int)Math.Ceiling(pnlMainGrid.ClientSize.Height * zoomMultipliers[zoomComboBox.SelectedIndex, 1]);
                renderedRect.X = daGridScrollPosition.X;
                renderedRect.Y = daGridScrollPosition.Y;
                renderedRect.Width = (int)Math.Ceiling(DaGrid.Width * zoomMultipliers[zoomComboBox.SelectedIndex, 1]);
                renderedRect.Height = (int)Math.Ceiling(DaGrid.Height * zoomMultipliers[zoomComboBox.SelectedIndex, 1]);

                daGridViewCenter = new PointF(
                    daGridScrollPosition.X + (DaGrid.Width / 2 * zoomMultipliers[zoomComboBox.SelectedIndex, 1]),
                    daGridScrollPosition.Y + (DaGrid.Height / 2 * zoomMultipliers[zoomComboBox.SelectedIndex, 1]));


                if (nc != null)
                {
                    nc.setBounds(renderedRect);
                }

                //    RectangleF bounds = nodeSteuerung.GetLineNodeBounds();
                //    float zoom = Math.Min(1.0f, Math.Min((float)thumbGrid.ClientSize.Width / bounds.Width, (float)thumbGrid.ClientSize.Height / bounds.Height));

                //    thumbGridClientRect = new Rectangle(
                //        (int)Math.Round((daGridScrollPosition.X - bounds.X) * zoom),
                //        (int)Math.Round((daGridScrollPosition.Y - bounds.Y) * zoom),
                //        (int)Math.Round(pnlMainGrid.ClientSize.Width * zoomMultipliers[zoomComboBox.SelectedIndex, 1] * zoom),
                //        (int)Math.Round(pnlMainGrid.ClientSize.Height * zoomMultipliers[zoomComboBox.SelectedIndex, 1] * zoom));

                //    lblScrollPosition.Text = "Canvas Location (dm): (" + daGridScrollPosition.X + ", " + daGridScrollPosition.Y + ") -> (" + (daGridScrollPosition.X + renderOptionsDaGrid.clippingRect.Width) + ", " + (daGridScrollPosition.Y + renderOptionsDaGrid.clippingRect.Height) + ")";

                //    UpdateConnectionsRenderCache();
            }
        }

        void DaGrid_MouseDown(object sender, MouseEventArgs e)
        {
            Vector2 clickedPosition = new Vector2(e.X, e.Y);
            clickedPosition *= zoomMultipliers[zoomComboBox.SelectedIndex, 1];
            clickedPosition += daGridScrollPosition;

            // Node Gedöns
            switch (e.Button)
            {
                case MouseButtons.Right:
                    if ((Control.ModifierKeys & Keys.Control) == Keys.Control)
                    {
                        #region Nodes löschen
                        //this.Cursor = Cursors.Default;
                        //// LineNode entfernen
                        //LineNode nodeToDelete = nodeSteuerung.GetLineNodeAt(clickedPosition);
                        //// checken ob gefunden
                        //if (nodeToDelete != null)
                        //{
                        //    if (selectedLineNodes.Contains(nodeToDelete))
                        //    {
                        //        selectedLineNodes.Remove(nodeToDelete);
                        //    }
                        //    nodeSteuerung.DeleteLineNode(nodeToDelete);
                        //}
                        #endregion
                    }
                    else
                    {
                        #region move main grid
                        howToDrag = DragNDrop.MOVE_MAIN_GRID;
                        daGridRubberband.Location = clickedPosition;
                        this.Cursor = Cursors.SizeAll;
                        #endregion
                    }

                    break;

                default:
                    break;
            }
            Invalidate(InvalidationLevel.ONLY_MAIN_CANVAS);
        }

        void DaGrid_MouseMove(object sender, MouseEventArgs e)
        {
            Vector2 clickedPosition = new Vector2(e.X, e.Y);
            clickedPosition *= zoomMultipliers[zoomComboBox.SelectedIndex, 1];
            clickedPosition += daGridScrollPosition;
            //lblMouseCoordinates.Text = "Current Mouse Coordinates (m): " + (clickedPosition * 0.1).ToString();

            this.Cursor = (howToDrag == DragNDrop.MOVE_MAIN_GRID) ? Cursors.SizeAll : Cursors.Default;

            //if (selectedLineNodes != null)
            //{
            switch (howToDrag)
            {
                case DragNDrop.MOVE_MAIN_GRID:
                    clickedPosition = new Vector2(e.X, e.Y);
                    clickedPosition *= zoomMultipliers[zoomComboBox.SelectedIndex, 1];
                    daGridScrollPosition = new Point((int)Math.Round(-clickedPosition.X + daGridRubberband.X), (int)Math.Round(-clickedPosition.Y + daGridRubberband.Y));
                    UpdateDaGridClippingRect();
                    Invalidate(InvalidationLevel.ONLY_MAIN_CANVAS);
                    break;
                default:
                    break;
            }
            //}
        }

        void DaGrid_MouseUp(object sender, MouseEventArgs e)
        {
            Vector2 clickedPosition = new Vector2(e.X, e.Y);
            clickedPosition *= zoomMultipliers[zoomComboBox.SelectedIndex, 1];
            clickedPosition += daGridScrollPosition;
            this.Cursor = Cursors.Default;

            switch (howToDrag)
            {
                case DragNDrop.MOVE_MAIN_GRID:
                    //thumbGrid.Invalidate();
                    break;
                default:
                    break;
            }

            howToDrag = DragNDrop.NONE;
            Invalidate(InvalidationLevel.ONLY_MAIN_CANVAS);
        }


        //void DrawZoom(Graphics g, Rectangle srcRect)
        //{
        //    Rectangle dstRect = new Rectangle(0, 0, DaGrid.Width, DaGrid.Height);
        //    g.DrawImage(bmp, dstRect, srcRect, GraphicsUnit.Pixel);
        //}

        #endregion



        private void LoadOsmMap_old(string path)
        {
            LoadingForm.LoadingForm lf = new LoadingForm.LoadingForm();
            lf.Text = "Loading file '" + path + "'...";
            lf.Show();

            lf.SetupUpperProgress("Loading Document...", 5);

            XmlDocument xd = new XmlDocument();
            xd.Load(path);

            XmlNode mainNode = xd.SelectSingleNode("//osm");
            XmlNode bounds = xd.SelectSingleNode("//osm/bounds");

            if (bounds == null)
            {
                Debug.WriteLine("bounds null");
            }
            else
            {


                XmlNode minLonNode = bounds.Attributes.GetNamedItem("minlon");
                XmlNode maxLonNode = bounds.Attributes.GetNamedItem("maxlon");
                XmlNode minLatNode = bounds.Attributes.GetNamedItem("minlat");
                XmlNode maxLatNode = bounds.Attributes.GetNamedItem("maxlat");

                if (minLonNode != null)
                {
                    minLon = double.Parse(minLonNode.Value, CultureInfo.InvariantCulture);// / 1000;// 10000000;
                    boundsDefined = true;
                }
                else { minLon = 0; }

                if (maxLonNode != null)
                {
                    maxLon = double.Parse(maxLonNode.Value, CultureInfo.InvariantCulture);// / 1000;// 10000000;
                    boundsDefined = true;
                }
                else { maxLon = 0; }

                if (minLatNode != null)
                {
                    minLat = double.Parse(minLatNode.Value, CultureInfo.InvariantCulture);// / 1000;// 10000000;
                    boundsDefined = true;
                }
                else { minLat = 0; }

                if (maxLatNode != null)
                {
                    maxLat = double.Parse(maxLatNode.Value, CultureInfo.InvariantCulture);// / 1000;// 10000000;
                    boundsDefined = true;
                }
                else { maxLat = 0; }

                UpdateDaGridClippingRect();


                Debug.WriteLine("minLong maxLat: " + minLon + ", " + maxLat);


                lf.StepUpperProgress("Parsing Nodes...");
                XmlNodeList xnlLineNode = xd.SelectNodes("//osm/node");
                lf.SetupLowerProgress("Parsing Nodes", xnlLineNode.Count - 1);

                Stopwatch sw = Stopwatch.StartNew();
                foreach (XmlNode aXmlNode in xnlLineNode)
                {
                    // Node in einen TextReader packen
                    TextReader tr = new StringReader(aXmlNode.OuterXml);
                    // und Deserializen
                    XmlSerializer xs = new XmlSerializer(typeof(Node));
                    Node n = (Node)xs.Deserialize(tr);
                    n.latLonToPos(minLon, maxLat);

                    // ab in die Liste
                    nc._nodes.Add(n);

                    lf.StepLowerProgress();
                }
                sw.Stop();
                Console.WriteLine("Total query time: {0} ms", sw.ElapsedMilliseconds);


                lf.StepUpperProgress("Parsing Ways / Roads...");
                XmlNodeList xnlWayNode = xd.SelectNodes("//osm/way");
                lf.SetupLowerProgress("Parsing Ways", xnlWayNode.Count - 1);

                sw = Stopwatch.StartNew();
                foreach (XmlNode aXmlNode in xnlWayNode)
                //Parallel.ForEach(xnlWayNode, (XmlNode aXmlNode) =>
                {
                    XmlNodeList nds = aXmlNode.SelectNodes("nd");
                    XmlNode onewayTag = aXmlNode.SelectSingleNode("tag[@k='oneway']");
                    XmlNode highwayTag = aXmlNode.SelectSingleNode("tag[@k='highway']");
                    XmlNode numlanesTag = aXmlNode.SelectSingleNode("tag[@k='lanes']");

                    List<XmlNode> lnd = new List<XmlNode>();

                    foreach (XmlNode nd in nds)
                    {
                        lnd.Add(nd);
                    }

                    if (onewayTag != null)
                    {
                        string oneway = onewayTag.Attributes.GetNamedItem("v").Value;

                        //if (oneway == "-1")
                        //{
                        //    makeWaySegment_old(lnd, highwayTag, numlanesTag, oneway);
                        //}
                        //else
                        //{
                        makeWaySegment_old(lnd, highwayTag, numlanesTag, oneway);
                        //}
                    }
                    else
                    {
                        makeWaySegment_old(lnd, highwayTag, numlanesTag, "");
                    }

                    lf.StepLowerProgress();

                }

                lf.StepUpperProgress("Search segment connection...");
                lf.SetupLowerProgress("Search segment connectio", nc.segments.Count - 1);

                for (int i = 0; i < nc.segments.Count; i++)
                {
                    nc.segments[i].nextSegment = nc.segments.FindAll(x => x.startNode == nc.segments[i].endNode);
                    nc.segments[i].prevSegment = nc.segments.FindAll(x => x.endNode == nc.segments[i].startNode);

                    lf.StepLowerProgress();
                }

                //});
                sw.Stop();
                Console.WriteLine("Total query time: {0} ms", sw.ElapsedMilliseconds);

                #region manually generate route

                lf.StepUpperProgress("Manually Generate Routes...");

                // TODO find by segment ID for route addition
                // _route.segments.Find(x => x.Id == segmentToAddID)

                Debug.WriteLine("Segment Count" + nc.segments.Count);
                manuallyAddRoute();

                lf.StepUpperProgress("Done");
                lf.ShowLog();

                lf.Close();
                lf = null;



                #endregion

                #region draw road as background
                //bmp = new Bitmap((int)Math.Ceiling((maxLon - minLon) * 111111), (int)Math.Ceiling((maxLat - minLat) * 111111));
                //using(Graphics g = Graphics.FromImage(bmp))
                //{
                //    g.SmoothingMode = SmoothingMode.HighQuality;
                //    g.InterpolationMode = InterpolationMode.HighQualityBicubic;

                //    foreach (WaySegment ws in nc.Segments)
                //    {
                //        ws.Draw(g);
                //    }
                //}
                //Invalidate(InvalidationLevel.MAIN_CANVAS_AND_TIMELINE);
                #endregion
            }


        }



        private void LoadOsmMap(string path)
        {
            LoadingForm.LoadingForm lf = new LoadingForm.LoadingForm();
            lf.Text = "Loading file '" + path + "'...";
            lf.Show();

            lf.SetupUpperProgress("Loading Document...", 5);
            lf.StepUpperProgress("Parsing Header...");

            XDocument xd = new XDocument();
            xd = XDocument.Load(path);
            XElement mainNode = xd.Element("osm");

            XElement bounds = mainNode.Element("bounds");

            // Check if there's a bounds or not
            // We need boundary for projecting lat lon to cartesian coord
            if (bounds == null)
            {
                Debug.WriteLine("bounds null");
            }
            else
            {
                minLon = double.Parse(bounds.Attribute("minlon").Value, CultureInfo.InvariantCulture);
                maxLon = double.Parse(bounds.Attribute("maxlon").Value, CultureInfo.InvariantCulture);
                minLat = double.Parse(bounds.Attribute("minlat").Value, CultureInfo.InvariantCulture);// / 1000;// 10000000;                
                maxLat = double.Parse(bounds.Attribute("maxlat").Value, CultureInfo.InvariantCulture);// / 1000;// 10000000;
                boundsDefined = true;

                //nc.setBounds(minLon, maxLon, minLat, maxLat);

                Debug.WriteLine("minLong maxLat: " + minLon + ", " + maxLat);


                lf.StepUpperProgress("Searching nodes in highways...");

                // Find only <node> referred by <way>
                // Make sure to only include <way> with highway tag
                // Also make sure no duplicate <node>

                // This code lists <node> id
                var query = xd.Descendants("way")
                            .Where(p => p.Elements("tag")
                                .Any(c => (string)c.Attribute("k") == "highway")
                            ).Elements("nd")
                            .GroupBy(i => i.Attribute("ref").Value)
                            .Select(g => g.Key)
                            .ToList();

                //var query = from way in xd.Descendants("way")
                //            where way.Elements("tag").Any(c => (string)c.Attribute("k") == "highway")
                //            select way.Elements("nd")
                //            group i by i.Attribute("ref").Value


                //var query = mainNode.Elements("node").Select(g => (string) g.Attribute("id")).ToList();
                lf.StepUpperProgress("Parsing nodes...");

                lf.SetupLowerProgress("Parsing nodes...", query.Count() - 1);
                Stopwatch sw = Stopwatch.StartNew();

                // Deserialize XML for each <node> in query
                // Find node by id, create new node, calculate position
                for (int i = 0; i < query.Count(); i++)
                {
                    Node nd = (from node in mainNode.Elements("node")
                               where node.Attribute("id").Value == query[i]
                               select new Node()
                               {
                                   Id = Convert.ToInt64(node.Attribute("id").Value),
                                   Lat = double.Parse(node.Attribute("lat").Value, CultureInfo.InvariantCulture),// / 10000000,
                                   Long = double.Parse(node.Attribute("lon").Value, CultureInfo.InvariantCulture)// / 10000000
                               }).Single();

                    nd.latLonToPos(minLon, maxLat);
                    nc._nodes.Add(nd);

                    lf.StepLowerProgress();
                }
                sw.Stop();
                Console.WriteLine("Total query time: {0} ms", sw.ElapsedMilliseconds);

                lf.StepUpperProgress("Parsing ways / roads...");

                List<XElement> wayQuery = xd.Descendants("way")
                                .Where(p => p.Elements("tag")
                                    .Any(c => (string)c.Attribute("k") == "highway")
                                ).ToList();

                //List<XElement> wayQuery = mainNode.Elements("way").ToList();
                lf.SetupLowerProgress("Parsing ways...", wayQuery.Count() - 1);
                sw = Stopwatch.StartNew();

                foreach (XElement way in wayQuery)
                {
                    List<XElement> nodeQuery = way.Elements("nd").ToList();
                    //Debug.WriteLine("query " + wayQuery[i] + " elem" + nodeQuery);

                    var oneway = from tag in way.Elements("tag")
                                 where tag.Attribute("k").Value == "oneway"
                                 select (string)tag.Attribute("v").Value;

                    if (oneway.Count() > 0)
                    {
                        if (oneway.Single() == "-1")
                        {
                            makeWaySegment(nodeQuery, false);
                        }
                        else
                        {
                            makeWaySegment(nodeQuery, true);
                        }
                    }
                    else
                    {
                        makeWaySegment(nodeQuery, true);
                    }

                    //Debug.WriteLine("values" + oneway);
                    lf.StepLowerProgress();
                }

                sw.Stop();
                Console.WriteLine("Total query time: {0} ms", sw.ElapsedMilliseconds);


            }

            lf.StepUpperProgress("Done");
            lf.ShowLog();

            lf.Close();
            lf = null;
        }

        private void makeWaySegment_old(List<XmlNode> lnd, XmlNode highwayTag, XmlNode numlanesTag, string oneway)
        {
            #region road type and lanes
            string highway;
            int numlanes;

            if (highwayTag != null) { highway = highwayTag.Attributes.GetNamedItem("v").Value; }
            else { highway = ""; }

            if (numlanesTag != null) { numlanes = int.Parse(numlanesTag.Attributes.GetNamedItem("v").Value); }
            else { numlanes = -1; }
            #endregion

            #region new approach
            if (oneway == "yes")        // Oneway Forward
            {
                for (int i = 0; i < lnd.Count - 1; i++)
                {

                    long ndId;
                    XmlNode ndIdNode = lnd[i].Attributes.GetNamedItem("ref");
                    //if (ndIdNode != null)
                    ndId = long.Parse(ndIdNode.Value);
                    //else
                    //    ndId = 0;

                    long ndNextId;
                    XmlNode ndIdNextNode = lnd[i + 1].Attributes.GetNamedItem("ref");
                    //if (ndIdNextNode != null)
                    ndNextId = long.Parse(ndIdNextNode.Value);
                    //else
                    //    ndNextId = 0;

                    if ((nc._nodes.Find(x => x.Id == ndId) != null) && (nc._nodes.Find(y => y.Id == ndNextId) != null))
                    {
                        nc.segments.Add(new RoadSegment(nc._nodes.Find(x => x.Id == ndId), nc._nodes.Find(y => y.Id == ndNextId), numlanes, highway, oneway));
                    }
                }
            }
            else if (oneway == "-1") // Oneway Reverse
            {
                for (int i = lnd.Count - 1; i > 0; i--)
                {

                    long ndId;
                    XmlNode ndIdNode = lnd[i].Attributes.GetNamedItem("ref");
                    if (ndIdNode != null)
                        ndId = long.Parse(ndIdNode.Value);
                    else
                        ndId = 0;

                    long ndNextId;
                    XmlNode ndIdNextNode = lnd[i - 1].Attributes.GetNamedItem("ref");
                    if (ndIdNextNode != null)
                        ndNextId = long.Parse(ndIdNextNode.Value);
                    else
                        ndNextId = 0;

                    if ((nc._nodes.Find(x => x.Id == ndId) != null) && (nc._nodes.Find(y => y.Id == ndNextId) != null))
                    {
                        nc.segments.Add(new RoadSegment(nc._nodes.Find(x => x.Id == ndId), nc._nodes.Find(y => y.Id == ndNextId), numlanes, highway, oneway));
                    }
                }
            }
            else                     // Two Way
            {
                for (int i = 0; i < lnd.Count - 1; i++)
                {
                    long ndId;
                    XmlNode ndIdNode = lnd[i].Attributes.GetNamedItem("ref");
                    if (ndIdNode != null)
                        ndId = long.Parse(ndIdNode.Value);
                    else
                        ndId = 0;

                    long ndNextId;
                    XmlNode ndIdNextNode = lnd[i + 1].Attributes.GetNamedItem("ref");
                    if (ndIdNextNode != null)
                        ndNextId = long.Parse(ndIdNextNode.Value);
                    else
                        ndNextId = 0;

                    RoadSegment tempSegment;

                    if ((nc._nodes.Find(x => x.Id == ndId) != null) && (nc._nodes.Find(y => y.Id == ndNextId) != null))
                    {
                        tempSegment = new RoadSegment(nc._nodes.Find(x => x.Id == ndId), nc._nodes.Find(y => y.Id == ndNextId), numlanes, highway, oneway);

                        int lanePerDirection = (int)tempSegment.lanes.Count / 2;
                        double distanceShift = (double)(tempSegment.lanes.Count / (double)4) * (double)tempSegment.laneWidth;

                        if (lanePerDirection < 1)
                            lanePerDirection = 1;

                        nc.segments.Add(generateShiftedSegment(tempSegment, distanceShift, lanePerDirection, tempSegment.Highway, true));
                        nc.segments.Add(generateShiftedSegment(tempSegment, -distanceShift, lanePerDirection, tempSegment.Highway, false));

                    }

                }
            }
            #endregion


            #region prev approach
            //if (oneway != "-1")
            //{
            //    for (int i = 0; i < lnd.Count - 1; i++)
            //    {

            //        long ndId;
            //        XmlNode ndIdNode = lnd[i].Attributes.GetNamedItem("ref");
            //        if (ndIdNode != null)
            //            ndId = long.Parse(ndIdNode.Value);
            //        else
            //            ndId = 0;

            //        long ndNextId;
            //        XmlNode ndIdNextNode = lnd[i + 1].Attributes.GetNamedItem("ref");
            //        if (ndIdNextNode != null)
            //            ndNextId = long.Parse(ndIdNextNode.Value);
            //        else
            //            ndNextId = 0;

            //        if ((nc._nodes.Find(x => x.Id == ndId) != null) && (nc._nodes.Find(y => y.Id == ndNextId) != null))
            //        {
            //            nc.segments.Add(new RoadSegment(nc._nodes.Find(x => x.Id == ndId), nc._nodes.Find(y => y.Id == ndNextId), numlanes, highway, oneway));
            //        }
            //    }
            //}
            //else // Oneway: Reverse
            //{
            //    for (int i = lnd.Count - 1; i > 0; i--)
            //    {

            //        long ndId;
            //        XmlNode ndIdNode = lnd[i].Attributes.GetNamedItem("ref");
            //        if (ndIdNode != null)
            //            ndId = long.Parse(ndIdNode.Value);
            //        else
            //            ndId = 0;

            //        long ndNextId;
            //        XmlNode ndIdNextNode = lnd[i - 1].Attributes.GetNamedItem("ref");
            //        if (ndIdNextNode != null)
            //            ndNextId = long.Parse(ndIdNextNode.Value);
            //        else
            //            ndNextId = 0;

            //        if ((nc._nodes.Find(x => x.Id == ndId) != null) && (nc._nodes.Find(y => y.Id == ndNextId) != null))
            //        {
            //            nc.segments.Add(new RoadSegment(nc._nodes.Find(x => x.Id == ndId), nc._nodes.Find(y => y.Id == ndNextId), numlanes, highway, oneway));
            //        }
            //    }
            //}
            #endregion
        }

        RoadSegment generateShiftedSegment(RoadSegment oriSegment, double distance, int numlanes, string highway, Boolean forward)
        {
            double angle = (Math.PI / 2) - Vector2.AngleBetween(oriSegment.startNode.Position, oriSegment.endNode.Position);
            Vector2 shift = new Vector2(distance * Math.Cos(angle), distance * Math.Sin(angle));

            Node newStart = new Node(new Vector2(oriSegment.startNode.Position.X + shift.X, oriSegment.startNode.Position.Y - shift.Y));
            Node newEnd = new Node(new Vector2(oriSegment.endNode.Position.X + shift.X, oriSegment.endNode.Position.Y - shift.Y));

            RoadSegment toReturn;

            if (forward)
            {
                toReturn = new RoadSegment(newStart, newEnd, numlanes, highway, "yes");
            }
            else
            {
                toReturn = new RoadSegment(newEnd, newStart, numlanes, highway, "yes");
            }

            return toReturn;
        }


        private void makeWaySegment(List<XElement> nodeQuery, Boolean forward)
        {
            if (forward)
            {
                for (int i = 0; i < nodeQuery.Count() - 1; i++)
                {
                    long ndCurrId = Convert.ToInt64(nodeQuery[i].Attribute("ref").Value);
                    long ndNextId = Convert.ToInt64(nodeQuery[i + 1].Attribute("ref").Value);

                    if ((nc._nodes.Find(x => x.Id == ndCurrId) != null) && (nc._nodes.Find(y => y.Id == ndNextId) != null))
                    {
                        //nc.segments.Add(new RoadSegment(nc._nodes.Find(x => x.Id == ndCurrId), nc._nodes.Find(y => y.Id == ndNextId)));
                    }
                }
            }
            else
            {
                for (int i = nodeQuery.Count - 1; i > 0; i--)
                {
                    long ndCurrId = Convert.ToInt64(nodeQuery[i].Attribute("ref").Value);
                    long ndNextId = Convert.ToInt64(nodeQuery[i - 1].Attribute("ref").Value);

                    if ((nc._nodes.Find(x => x.Id == ndCurrId) != null) && (nc._nodes.Find(y => y.Id == ndNextId) != null))
                    {
                        //nc.segments.Add(new RoadSegment(nc._nodes.Find(x => x.Id == ndCurrId), nc._nodes.Find(y => y.Id == ndNextId)));
                    }
                }
            }
        }


        private void tempLoadButton_Click(object sender, EventArgs e)
        {
            #region Load File
            using (OpenFileDialog ofd = new OpenFileDialog())
            {
                ofd.InitialDirectory = Application.ExecutablePath;
                ofd.AddExtension = true;
                ofd.DefaultExt = @".xml";
                ofd.Filter = @"OpenStreetMap|*.osm";

                if (ofd.ShowDialog() == DialogResult.OK)
                {
                    GlobalTime.Instance.Reset();
                    nc.Clear();
                    nc.Load();

                    LoadOsmMap_old(ofd.FileName);
                    //LoadOsmMap(ofd.FileName);
                }
            }
            #endregion


            //#region Longitudinal Model Test
            //GlobalTime.Instance.Reset();
            //nc.Clear();

            //nc._nodes.Add(new Node(new Vector2(0, 200)));
            //nc._nodes.Add(new Node(new Vector2(2000, 200)));

            //nc.segments.Add(new WaySegment(nc._nodes[0], nc._nodes[1]));
            //_route = new List<WaySegment>();
            //_route.Add(nc.segments[0]);

            //_route[0].vehicles.Add(new IVehicle(
            //    _route[0],
            //    Color.FromArgb(rnd.Next(0, 256), rnd.Next(0, 256), rnd.Next(0, 256)),
            //    _route));

            //_route[0].vehicles[0]._physics.targetVelocity = 7;
            //_route[0].vehicles[0].distance = 100;

            //_route[0].vehicles.Add(new IVehicle(
            //    _route[0],
            //    Color.FromArgb(rnd.Next(0, 256), rnd.Next(0, 256), rnd.Next(0, 256)),
            //    _route));

            //_route[0].vehicles[1]._physics.targetVelocity = 13;
            //_route[0].vehicles[1].distance = 50;

            //_route[0].vehicles.Add(new IVehicle(
            //    _route[0],
            //    Color.FromArgb(rnd.Next(0, 256), rnd.Next(0, 256), rnd.Next(0, 256)),
            //    _route));

            //_route[0].vehicles[2]._physics.targetVelocity = 10;
            //_route[0].vehicles[2].distance = 0;

            //#endregion


            Invalidate(InvalidationLevel.MAIN_CANVAS_AND_TIMELINE);
        }

        private void speedComboBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            timerSimulation.Interval = (int)(1000 / (int)_temp_stepsPerSeconds / speedMultipliers[speedComboBox.SelectedIndex]);
            Debug.WriteLine("timerSimulation Interval " + timerSimulation.Interval + ", " + speedMultipliers[speedComboBox.SelectedIndex]);
        }


        private void zoomComboBox_SelectedIndexChanged(object sender, EventArgs e)
        {

            // neue Autoscrollposition berechnen und setzen
            daGridScrollPosition = new Point(
                (int)Math.Round(daGridViewCenter.X - (DaGrid.Width / 2 * zoomMultipliers[zoomComboBox.SelectedIndex, 1])),
                (int)Math.Round(daGridViewCenter.Y - (DaGrid.Height / 2 * zoomMultipliers[zoomComboBox.SelectedIndex, 1])));

            // Bitmap umrechnen:
            //UpdateBackgroundImage();

            UpdateDaGridClippingRect();
            //thumbGrid.Invalidate();
            DaGrid.Invalidate();
        }

        private void buttonTLightTemp_Click(object sender, EventArgs e)
        {
            switch (nc.segments.Find(x => x.Id == 2918).endNode.tLight.trafficLightState)
            {
                case TrafficLight.State.GREEN:
                    nc.segments.Find(x => x.Id == 2918).endNode.tLight.SwitchToRed();
                    nc.segments.Find(x => x.Id == 25163).endNode.tLight.SwitchToRed();
                    break;
                case TrafficLight.State.RED:
                    nc.segments.Find(x => x.Id == 2918).endNode.tLight.SwitchToGreen();
                    nc.segments.Find(x => x.Id == 25163).endNode.tLight.SwitchToGreen();
                    break;
            }

            Invalidate(InvalidationLevel.MAIN_CANVAS_AND_TIMELINE);
        }




        private void manuallyAddRoute()
        {
            _route = new List<RoadSegment>();  //Dayeuhkolot-Buahbatu
            _route2 = new List<RoadSegment>(); //Buahbatu-Samsat
            //_route3 = new List<RoadSegment>(); //Soetta-Buah Batu
            //_route4 = new List<RoadSegment>(); //Samsat-Gatsu

            _route5 = new List<RoadSegment>(); //Buahbatu - Dayeuhkolot
            _route6 = new List<RoadSegment>(); //Samsat - Buahbatu

            // Route 1 : Dayeuhkolot-Buahbatu
            for (int i = 37364; i < 37368; i++)
            {
                _route.Add(nc.segments.Find(x => x.Id == i));
            }

            // Route 2 : Buahbatu-Samsat
            for (int i = 25071; i < 25077; i++)
            {
                _route2.Add(nc.segments.Find(x => x.Id == i));
            }
            _route2.Add(nc.segments.Find(x => x.Id == 33228));
            _route2.Add(nc.segments.Find(x => x.Id == 33229));
            for (int i = 32013; i < 32016; i++)
            {
                _route2.Add(nc.segments.Find(x => x.Id == i));
            }
            // nc.segments.Find(x => x.Id == 2918).endNode.tLight = new TrafficLight();
            
            /***************
            // Route 3 : Soetta-Buahbatu
            for (int i = 25372; i < 25376; i++)
            {
                _route3.Add(nc.segments.Find(x => x.Id == i));
            }
            //nc.segments.Find(x => x.Id == 25163).endNode.tLight = new TrafficLight();

            // Route 4 : Samsat-Gatsu
            for (int i = 33875; i < 33886; i++)
            {
                _route4.Add(nc.segments.Find(x => x.Id == i));
            }
            
            //Route 5 : Buahbatu - Dayeuhkolot
            _route5.Add(nc.segments.Find(x => x.Id == 25079));
            _route5.Add(nc.segments.Find(x => x.Id == 25080));
            _route5.Add(nc.segments.Find(x => x.Id == 25081));
            
            //Route 6 : Samsat - Buahbatu
            for (int i = 28285; i < 28287; i++)
            {
                _route6.Add(nc.segments.Find(x => x.Id == i));
            }
            for (int i = 32219; i < 32226; i++)
            {
                _route6.Add(nc.segments.Find(x => x.Id == i));
            }
            //nc.segments.Find(x => x.Id == 32227).endNode.tLight = new TrafficLight();
            */
        }

        #region oldRoute
        private void oldRoute()
        {
            _route = new List<RoadSegment>();

            //_route[0].lanes[rnd.Next(0, _route[0].lanes.Count)].vehicles.Add(new IVehicle(
            //            _route[0],
            //            Color.FromArgb(rnd.Next(64, 200), rnd.Next(64, 200), rnd.Next(64, 200)),
            //            _route));

            //_route.Add(nc.segments[0]);
            //_route.Add(nc.segments[1]);
            //_route.Add(nc.segments[2]);
            //_route.Add(nc.segments[39]);
            //_route.Add(nc.segments[264]);
            //_route.Add(nc.segments[262]);
            //_route.Add(nc.segments[265]);
            //_route.Add(nc.segments[266]);
            //_route.Add(nc.segments[374]);
            //_route.Add(nc.segments[349]);
            //_route.Add(nc.segments[47]);
            //_route.Add(nc.segments[350]);
            //_route.Add(nc.segments[351]);
            //_route.Add(nc.segments[352]);
            //_route.Add(nc.segments[353]);
            //_route.Add(nc.segments[354]);
            //_route.Add(nc.segments[355]);
            //_route.Add(nc.segments[356]);
            //_route.Add(nc.segments[357]);
            //_route.Add(nc.segments[358]);
            //_route.Add(nc.segments[359]);
            //_route.Add(nc.segments[360]);
            //_route.Add(nc.segments[361]);
            //_route.Add(nc.segments[362]);
            //_route.Add(nc.segments[363]);
            //_route.Add(nc.segments[364]);

            //nc.segments[374].endNode.tLight = new TrafficLight();
            //_route[0].vehicles.Add(new IVehicle(
            //    _route[0],
            //    Color.FromArgb(rnd.Next(0, 256), rnd.Next(0, 256), rnd.Next(0, 256)),
            //    _route));

            ////_route[0].vehicles[0].dumb = true;
            //_route[0].vehicles[0].distance = 20;

            //_route[0].vehicles.Add(new IVehicle(
            //    _route[0],
            //    Color.FromArgb(rnd.Next(0, 256), rnd.Next(0, 256), rnd.Next(0, 256)),
            //    _route));

            //_route[0].vehicles[1].distance = 10;

            //_route[0].vehicles.Add(new IVehicle(
            //    _route[0],
            //    Color.FromArgb(rnd.Next(0, 256), rnd.Next(0, 256), rnd.Next(0, 256)),
            //    _route));

            //_route[0].vehicles[2].distance = 0;
        }
        #endregion

        private void generateVehicles()
        {
            #region tempVehGenerate
            if ((timeMod % 36) == 0.0)
            {
                if ((vehCount % 2) == 0)
                {
                    int laneidx = rnd.Next(0, _route[0].lanes.Count);
                    int vehType = rnd.Next(0, 2);
                    IVehicle v = null;

                    //route 1 Dayeuhkolot-Buahbatu
                    if (vehType == 0)
                    {
                        v = new Car(_route[0], laneidx, _route);
                    }
                    else if (vehType == 1)
                    {
                        v = new Bus(_route[0], laneidx, _route);
                    }
                    else
                    {
                        v = new Truck(_route[0], laneidx, _route);
                    }

                    _route[0].lanes[laneidx].vehicles.Add(v);
                    activeVehicles++;
                    //-------route 1--------

                    //route 2 Buahbatu-Samsat
                    laneidx = rnd.Next(0, _route2[0].lanes.Count);
                    vehType = rnd.Next(0, 2);
                    if (vehType == 0)
                    {
                        v = new Car(_route2[0], laneidx, _route2);
                    }
                    else if (vehType == 1)
                    {
                        v = new Bus(_route2[0], laneidx, _route2);
                    }
                    else
                    {
                        v = new Truck(_route2[0], laneidx, _route2);
                    }

                    _route2[0].lanes[laneidx].vehicles.Add(v);
                    activeVehicles++;
                    //-------route 2--------

                    /********************
                    //route 3 Soetta-Buahbatu
                    if (vehType == 0)
                    {
                        v = new Car(_route3[0], laneidx, _route3);
                    }
                    else if (vehType == 1)
                    {
                        v = new Bus(_route3[0], laneidx, _route3);
                    }
                    else
                    {
                        v = new Truck(_route3[0], laneidx, _route3);
                    }

                    _route3[0].lanes[laneidx].vehicles.Add(v);
                    activeVehicles++;
                    //---route 3---

                    //route 4 Samsat-Gatsu
                    if (vehType == 0)
                    {
                        v = new Car(_route4[0], laneidx, _route4);
                    }
                    else if (vehType == 1)
                    {
                        v = new Bus(_route4[0], laneidx, _route4);
                    }
                    else
                    {
                        v = new Truck(_route4[0], laneidx, _route4);
                    }

                    _route4[0].lanes[laneidx].vehicles.Add(v);
                    activeVehicles++;
                    //---route 4---
                    */
                }
                else
                {
                    /********************
                    int laneidx = rnd.Next(0, _route2[0].lanes.Count);
                    int vehType = rnd.Next(0, 2);
                    IVehicle v = null;

                    //int laneidx = rnd.Next(0, _route2[0].lanes.Count);
                    //_route2[0].lanes[laneidx].vehicles.Add(new IVehicle(
                    //    _route2[0], laneidx,
                    //    Color.FromArgb(rnd.Next(64, 200), rnd.Next(64, 200), rnd.Next(64, 200)),
                    //    _route2));
                    //activeVehicles++;

                    //laneidx = rnd.Next(0, _route3[0].lanes.Count);
                    //_route3[0].lanes[laneidx].vehicles.Add(new IVehicle(
                    //    _route3[0], laneidx,
                    //    Color.FromArgb(rnd.Next(64, 200), rnd.Next(64, 200), rnd.Next(64, 200)),
                    //    _route3));
                    //activeVehicles++;

                    //route 5 Buahbatu-Dayeuhkolot
                    if (vehType == 0)
                    {
                        v = new Car(_route5[0], laneidx, _route5);
                    }
                    else if (vehType == 1)
                    {
                        v = new Bus(_route5[0], laneidx, _route5);
                    }
                    else
                    {
                        v = new Truck(_route5[0], laneidx, _route5);
                    }

                    _route5[0].lanes[laneidx].vehicles.Add(v);
                    activeVehicles++;
                    //---route 5---

                    //route 6 Samsat-Buahbatu
                    laneidx = rnd.Next(0, _route6[0].lanes.Count);
                    vehType = rnd.Next(0, 2);
                    

                    if (vehType == 0)
                    {
                        v = new Car(_route6[0], laneidx, _route6);
                    }
                    else if (vehType == 1)
                    {
                        v = new Bus(_route6[0], laneidx, _route6);
                    }
                    else
                    {
                        v = new Truck(_route6[0], laneidx, _route6);
                    }

                    _route6[0].lanes[laneidx].vehicles.Add(v);
                    activeVehicles++;
                    //---route 6---
                    */

                    //laneidx = rnd.Next(0, _route6[0].lanes.Count);
                    //_route6[0].lanes[laneidx].vehicles.Add(new IVehicle(
                    //    _route6[0], laneidx,
                    //    Color.FromArgb(rnd.Next(64, 200), rnd.Next(64, 200), rnd.Next(64, 200)),
                    //    _route6));
                    //activeVehicles++;
                }
                vehCount++;
            }
            //Debug.WriteLine("VehCount " + vehCount);
            timeMod++;
            #endregion
        }
    }
}
