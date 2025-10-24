using SimTMDG.Database;
using SimTMDG.Database.Entity;
using SimTMDG.Road;
using SimTMDG.Time;
using SimTMDG.Tools;
using SimTMDG.Vehicle;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Globalization;
using System.IO;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Xml;
using System.Xml.Serialization;

namespace SimTMDG
{
    public partial class Main : Form
    {
        //Fields dan properti yang digunakan dalam kelas Main
        #region TEMP
        private double _temp_stepsPerSeconds = 10; //Mengatur langkah per detik (kecepatan) untuk simulasi
        private double _temp_simulationDuration = 15; //Durasi simulasi dalam detik
        private NodeControl nc;//Objek NodeControl untuk mengelola jaringan jalan dan node dalam simulasi

        private long _stopSegmentId1 = 37367; // Titik henti untuk _route (Dayeuhkolot-Buahbatu)
        private long _stopSegmentId2 = 25376; // Titik henti untuk _route2 (Buahbatu-Samsat)
        private long _stopSegmentId3 = 34268; // Titik henti untuk _route3 (Buahbatu-Dayeuhkolot)
        private long _stopSegmentId4 = 33890; // Titik henti untuk _route4 (Barat-Samsat)

        //List RoadSegment untuk menyimpan rute kendaraan
        private List<RoadSegment> _route;
        private List<RoadSegment> _route2;
        private List<RoadSegment> _route3;
        private List<RoadSegment> _route4;

        private int _queue_route1 = 0;
        private int _queue_route2 = 0;
        private int _queue_route3 = 0;
        private int _queue_route4 = 0;

        private List<IVehicle> _queuedVehiclesRoute1 = new List<IVehicle>();
        private List<IVehicle> _queuedVehiclesRoute2 = new List<IVehicle>();
        private List<IVehicle> _queuedVehiclesRoute3 = new List<IVehicle>();
        private List<IVehicle> _queuedVehiclesRoute4 = new List<IVehicle>();

        Random rnd = new Random();//Objek Random untuk menghasilkan angka acak
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
        [Serializable]
        public struct WindowSettings
        {
            public FormWindowState _windowState;
            public Point _position;
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

        // Helper function to check if a point is near a segment
        private bool IsPointNearSegment(Vector2 point, RoadSegment segment, double threshold = 10.0)
        {
            Vector2 a = segment.startNode.Position;
            Vector2 b = segment.endNode.Position;
            double distance = DistancePointToSegment(point, a, b);
            return distance < threshold;
        }

        // Calculate distance from point to line segment
        private double DistancePointToSegment(Vector2 p, Vector2 a, Vector2 b)
        {
            double dx = b.X - a.X;
            double dy = b.Y - a.Y;
            if (dx == 0 && dy == 0)
            {
                // a == b
                dx = p.X - a.X;
                dy = p.Y - a.Y;
                return Math.Sqrt(dx * dx + dy * dy);
            }

            double t = ((p.X - a.X) * dx + (p.Y - a.Y) * dy) / (dx * dx + dy * dy);
            t = Math.Max(0, Math.Min(1, t));
            double projX = a.X + t * dx;
            double projY = a.Y + t * dy;
            dx = p.X - projX;
            dy = p.Y - projY;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        public enum InvalidationLevel
        {
            ALL,
            ONLY_MAIN_CANVAS,
            MAIN_CANVAS_AND_TIMELINE
        }
        #endregion

        #region Variables / Properties
        private const int MaxActiveVehicles = 50;
        private bool simIsPlaying = false;
        private System.Diagnostics.Stopwatch renderStopwatch = new System.Diagnostics.Stopwatch();
        private System.Diagnostics.Stopwatch thinkStopwatch = new System.Diagnostics.Stopwatch();
        private DragNDrop howToDrag = DragNDrop.NONE;
        private Rectangle daGridRubberband;
        private Point daGridScrollPosition = new Point();
        private PointF daGridViewCenter = new Point();
        private Scheduler _scheduler;
        private MongoService _mongoService;

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
            InitializeComponent();
            speedComboBox.SelectedIndex = 0;
            zoomComboBox.SelectedIndex = 8;
            daGridScrollPosition = new Point(0, 0);
            renderedRect = new Rectangle();
            UpdateDaGridClippingRect();
            DaGrid.Dock = DockStyle.Fill;
            this.SetStyle(ControlStyles.OptimizedDoubleBuffer | ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint, true);

            this.nc = new NodeControl();

            string osmPath = @"C:\Users\ajico\Documents\Coding\Backend\c#_digtwin\SimTMDG-master\osm-map\bandung-city-wide.osm";
            LoadOsmMap(osmPath);

            this.Load += Main_Load;
        }

        public async void Main_Load(object sender, EventArgs e)
        {
            _mongoService = new MongoService();
            string cronTime = "*/5 * * * * *";

            _scheduler = new Scheduler(cronTime, async () =>
            {
                Debug.WriteLine($"[{DateTime.Now}] Cron job jalan!");

                if (this.InvokeRequired)
                {
                    this.Invoke(new Action(async () => await this.GenerateVehicleFromDb()));
                }
                else
                {
                    await this.GenerateVehicleFromDb();
                }
            });
        }

        #region timer
        private void timerSimulation_Tick(object sender, EventArgs e)
        {
            Task.Run(() =>
            {
                double tickLength = 1.0d / _temp_stepsPerSeconds;
                GlobalTime.Instance.Advance(tickLength);
                nc.Tick(tickLength);
                nc.Reset();

                this.Invoke((Action)(() =>
                {
                    Invalidate(InvalidationLevel.MAIN_CANVAS_AND_TIMELINE);
                }));
            });
        }

        #endregion

        #region UI event
        private void playButton_Click(object sender, EventArgs e)
        {
            if (!simIsPlaying)
            {
                playButton.Text = "Pause";
                _scheduler.Start();

            }
            else
            {
                playButton.Text = "Play";
                _scheduler.Stop();
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
            var legends = new List<(string Name, Color Color)>
            {
                ("Motorcycle", Color.FromArgb(17, 34, 78)),
                ("Car", Color.FromArgb(248, 123, 27)),
                ("Bus", Color.FromArgb(59, 151, 151)),
                ("Truck", Color.FromArgb(191, 9, 47))
            };

            e.Graphics.SmoothingMode = SmoothingMode.HighQuality;
            e.Graphics.InterpolationMode = InterpolationMode.HighQualityBicubic;

            renderStopwatch.Reset();
            renderStopwatch.Start();

            e.Graphics.Transform = new Matrix(
                zoomMultipliers[zoomComboBox.SelectedIndex, 0], 0,
                0, zoomMultipliers[zoomComboBox.SelectedIndex, 0],
                -daGridScrollPosition.X * zoomMultipliers[zoomComboBox.SelectedIndex, 0], -daGridScrollPosition.Y * zoomMultipliers[zoomComboBox.SelectedIndex, 0]);

            if (boundsDefined)
            {
                nc.Draw(e.Graphics, zoomComboBox.SelectedIndex);
            }

            renderStopwatch.Stop();

            e.Graphics.Transform = new Matrix(1, 0, 0, 1, 0, 0);
            e.Graphics.DrawString(
                "thinking time: " + thinkStopwatch.ElapsedMilliseconds + "ms, possible thoughts per second: " + ((thinkStopwatch.ElapsedMilliseconds != 0) ? (1000 / thinkStopwatch.ElapsedMilliseconds).ToString() : "-"),
                new Font("Arial", 10),
                new SolidBrush(Color.Black),
                8,
                8);

            e.Graphics.DrawString(
                "rendering time: " + renderStopwatch.ElapsedMilliseconds + "ms, possible fps: " + ((renderStopwatch.ElapsedMilliseconds != 0) ? (1000 / renderStopwatch.ElapsedMilliseconds).ToString() : "-"),
                new Font("Arial", 10),
                new SolidBrush(Color.Black),
                8,
                24);

            e.Graphics.DrawString(
                "Active Vehicles: " + nc.ActiveVehicleCount,
                new Font("Arial", 10),
                new SolidBrush(Color.Black),
                8,
                40);

            e.Graphics.DrawString(
                "Queue From Dayeuhkolot to Buah Batu: " + _queue_route1,
                new Font("Arial", 10),
                new SolidBrush(Color.Black),
                8,
                56);

            e.Graphics.DrawString(
                "Queue From Buah Batu to Samsat: " + _queue_route2,
                new Font("Arial", 10),
                new SolidBrush(Color.Black),
                8,
                72);

            e.Graphics.DrawString(
                "Queue From Buah Batu to Dayeuhkolot: " + _queue_route3,
                new Font("Arial", 10),
                new SolidBrush(Color.Black),
                8,
                88);

            e.Graphics.DrawString(
                "Queue From West to Samsat: " + _queue_route4,
                new Font("Arial", 10),
                new SolidBrush(Color.Black),
                8,
                104);

            int legendStartX = 8;
            int boxSize = 12;
            int spacingY = 20;
            int legendHeight = legends.Count * spacingY;
            int legendStartY = e.ClipRectangle.Height - legendHeight - 10;

            using (Font legendFont = new Font("Arial", 10))
            {
                e.Graphics.DrawString("Legend :", new Font("Arial", 10, FontStyle.Bold), Brushes.Black, legendStartX, legendStartY - 20);

                for (int i = 0; i < legends.Count; i++)
                {
                    var item = legends[i];
                    int y = legendStartY + i * spacingY;

                    e.Graphics.FillRectangle(new SolidBrush(item.Color), legendStartX, y, boxSize, boxSize);
                    e.Graphics.DrawString(item.Name, legendFont, Brushes.Black, legendStartX + boxSize + 8, y - 1);
                }
            }

        }

        private void Invalidate(InvalidationLevel il)
        { 
            base.Invalidate();
            switch (il)
            { 
                case InvalidationLevel.ALL: 
                    DaGrid.Invalidate(); 
                    break; 
                case InvalidationLevel.MAIN_CANVAS_AND_TIMELINE: 
                    DaGrid.Invalidate(); 
                    break; 
                case InvalidationLevel.ONLY_MAIN_CANVAS: DaGrid.Invalidate(); 
                    break; 
                default: 
                    break; 
            } 
        }

        private void tableLayoutPanel1_Paint(object sender, PaintEventArgs e)
        {

        }

        private void UpdateDaGridClippingRect()
        {
            if (zoomComboBox.SelectedIndex >= 0)
            {
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
            }
        }

        void DaGrid_MouseDown(object sender, MouseEventArgs e)
        {
            Vector2 clickedPosition = new Vector2(e.X, e.Y);
            clickedPosition *= zoomMultipliers[zoomComboBox.SelectedIndex, 1];
            clickedPosition += daGridScrollPosition;

            switch (e.Button)
            {
                case MouseButtons.Right:
                    if ((Control.ModifierKeys & Keys.Control) == Keys.Control)
                    {
                        // Node deletion logic (if any)
                    }
                    else
                    {
                        howToDrag = DragNDrop.MOVE_MAIN_GRID;
                        daGridRubberband.Location = clickedPosition;
                        this.Cursor = Cursors.SizeAll;
                    }
                    break;

                //case MouseButtons.Left:
                //    try
                //    {
                //        RoadSegment clickedRoute = null;
                //        int routeIdx = -1;
                //        var allRoutes = new List<List<RoadSegment>> { this._route, this._route2, this._route3, this._route4 };

                //        foreach (var routeList in allRoutes)
                //        {
                //            if (routeList == null || routeList.Count == 0) continue;
                //            foreach (var seg in routeList)
                //            {
                //                if (IsPointNearSegment(clickedPosition, seg))
                //                {
                //                    clickedRoute = seg;
                //                    routeIdx = allRoutes.IndexOf(routeList);
                //                    break;
                //                }
                //            }
                //            if (clickedRoute != null) break;
                //        }

                //        if (clickedRoute != null)
                //        {
                //            // Batasi jumlah kendaraan per lane
                //            int maxVehiclesPerLane = 50;
                //            int laneidx = rnd.Next(0, clickedRoute.lanes.Count);

                //            if (clickedRoute.lanes == null || clickedRoute.lanes.Count == 0)
                //            {
                //                MessageBox.Show("Segment tidak memiliki lane yang valid.");
                //                break;
                //            }

                //            if (clickedRoute.lanes[laneidx].vehicles == null)
                //                clickedRoute.lanes[laneidx].vehicles = new List<IVehicle>();

                //            if (clickedRoute.lanes[laneidx].vehicles.Count >= maxVehiclesPerLane)
                //            {
                //                MessageBox.Show("Kendaraan di lane ini sudah terlalu banyak.");
                //                break;
                //            }

                //            int vehType = rnd.Next(0, 2);
                //            IVehicle v = null;
                //            var routeList = allRoutes[routeIdx];

                //            switch (vehType)
                //            {
                //                case 0: v = new Car(clickedRoute, laneidx, routeList); break;
                //                case 1: v = new Bus(clickedRoute, laneidx, routeList); break;
                //                default: v = new Truck(clickedRoute, laneidx, routeList); break;
                //            }

                //            clickedRoute.lanes[laneidx].vehicles.Add(v);
                //            activeVehicles++;

                //            Invalidate(InvalidationLevel.ONLY_MAIN_CANVAS);
                //        }
                //    }
                //    catch (Exception ex)
                //    {
                //        MessageBox.Show("Error saat generate kendaraan: " + ex.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                //    }
                //    break;

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

            this.Cursor = (howToDrag == DragNDrop.MOVE_MAIN_GRID) ? Cursors.SizeAll : Cursors.Default;

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
                    break;
                default:
                    break;
            }

            howToDrag = DragNDrop.NONE;
            Invalidate(InvalidationLevel.ONLY_MAIN_CANVAS);
        }
        #endregion

        private void LoadOsmMap(string path)
        {
            LoadingForm.LoadingForm loadingForm = new LoadingForm.LoadingForm();
            loadingForm.Text = $"Loading file '{path}'...";
            loadingForm.Show();
            loadingForm.SetupUpperProgress("Memuat Dokumen...", 5);

            XmlDocument xmlDoc = new XmlDocument();
            xmlDoc.Load(path);

            XmlNode osmNode = xmlDoc.SelectSingleNode("//osm");
            XmlNode boundsNode = xmlDoc.SelectSingleNode("//osm/bounds");

            if (boundsNode == null)
            {
                Debug.WriteLine("bounds null");
                loadingForm.Close();
                return;
            }

            minLon = boundsNode.Attributes["minlon"] != null ? double.Parse(boundsNode.Attributes["minlon"].Value, CultureInfo.InvariantCulture) : 0;
            maxLon = boundsNode.Attributes["maxlon"] != null ? double.Parse(boundsNode.Attributes["maxlon"].Value, CultureInfo.InvariantCulture) : 0;
            minLat = boundsNode.Attributes["minlat"] != null ? double.Parse(boundsNode.Attributes["minlat"].Value, CultureInfo.InvariantCulture) : 0;
            maxLat = boundsNode.Attributes["maxlat"] != null ? double.Parse(boundsNode.Attributes["maxlat"].Value, CultureInfo.InvariantCulture) : 0;
            boundsDefined = true;

            UpdateDaGridClippingRect();
            Debug.WriteLine($"minLon: {minLon}, maxLat: {maxLat}");

            loadingForm.StepUpperProgress("Parsing Node...");
            XmlNodeList nodeList = xmlDoc.SelectNodes("//osm/node");
            loadingForm.SetupLowerProgress("Parsing Node", nodeList.Count);

            Stopwatch sw = Stopwatch.StartNew();
            nc._nodes.Clear();
            foreach (XmlNode nodeXml in nodeList)
            {
                TextReader tr = new StringReader(nodeXml.OuterXml);
                XmlSerializer xs = new XmlSerializer(typeof(Node));
                Node node = (Node)xs.Deserialize(tr);
                node.latLonToPos(minLon, maxLat);
                nc._nodes.Add(node);
                loadingForm.StepLowerProgress();
            }
            sw.Stop();
            Console.WriteLine($"Waktu parsing node: {sw.ElapsedMilliseconds} ms");

            long centerNodeId = 2614138809;
            Node centerNode = this.nc._nodes.Find(n => n.Id == centerNodeId);
            if (centerNode != null)
            {
                Vector2 pos = centerNode.Position;
                daGridScrollPosition = new Point(
                    (int)(pos.X - (DaGrid.Width / 2 * zoomMultipliers[zoomComboBox.SelectedIndex, 1])),
                    (int)(pos.Y - (DaGrid.Height / 2 * zoomMultipliers[zoomComboBox.SelectedIndex, 1]))
                );
                UpdateDaGridClippingRect();
                DaGrid.Invalidate();
            }

            loadingForm.StepUpperProgress("Parsing Jalan...");
            XmlNodeList wayList = xmlDoc.SelectNodes("//osm/way");
            loadingForm.SetupLowerProgress("Parsing Jalan", wayList.Count);

            sw = Stopwatch.StartNew();
            nc.segments.Clear();
            foreach (XmlNode wayXml in wayList)
            {
                XmlNodeList nds = wayXml.SelectNodes("nd");
                XmlNode onewayTag = wayXml.SelectSingleNode("tag[@k='oneway']");
                XmlNode highwayTag = wayXml.SelectSingleNode("tag[@k='highway']");
                XmlNode numlanesTag = wayXml.SelectSingleNode("tag[@k='lanes']");

                List<XmlNode> nodeRefs = new List<XmlNode>();
                foreach (XmlNode nd in nds)
                    nodeRefs.Add(nd);

                string oneway = onewayTag != null ? onewayTag.Attributes["v"].Value : "";
                this.makeWaySegment(nodeRefs, highwayTag, numlanesTag, oneway);

                loadingForm.StepLowerProgress();
            }
            sw.Stop();
            Console.WriteLine($"Waktu parsing jalan: {sw.ElapsedMilliseconds} ms");

            loadingForm.StepUpperProgress("Mencari koneksi antar segmen...");
            loadingForm.SetupLowerProgress("Koneksi Segmen", nc.segments.Count);

            sw = Stopwatch.StartNew();
            for (int i = 0; i < nc.segments.Count; i++)
            {
                nc.segments[i].nextSegment = nc.segments.FindAll(x => x.startNode == nc.segments[i].endNode);
                nc.segments[i].prevSegment = nc.segments.FindAll(x => x.endNode == nc.segments[i].startNode);
                loadingForm.StepLowerProgress();
            }
            sw.Stop();
            Console.WriteLine($"Waktu koneksi segmen: {sw.ElapsedMilliseconds} ms");

            loadingForm.StepUpperProgress("Generate Rute Manual...");
            Debug.WriteLine($"Jumlah Segmen: {nc.segments.Count}");
            manuallyAddRoute();

            loadingForm.StepUpperProgress("Selesai");
            loadingForm.ShowLog();
            loadingForm.Close();
        }

        private void makeWaySegment(List<XmlNode> lnd, XmlNode highwayTag, XmlNode numlanesTag, string oneway)
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
            if (oneway == "yes")
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
            else
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

                        nc.segments.Add(this.GenerateShiftedSegment(tempSegment, distanceShift, lanePerDirection, tempSegment.Highway, true));
                        nc.segments.Add(this.GenerateShiftedSegment(tempSegment, -distanceShift, lanePerDirection, tempSegment.Highway, false));

                    }

                }
            }
            #endregion
        }

        RoadSegment GenerateShiftedSegment(RoadSegment oriSegment, double distance, int numlanes, string highway, Boolean forward)
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

                    this.LoadOsmMap(ofd.FileName);
                }
            }
            #endregion
            Invalidate(InvalidationLevel.MAIN_CANVAS_AND_TIMELINE);
        }

        private void speedComboBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            timerSimulation.Interval = (int)(1000 / (int)_temp_stepsPerSeconds / speedMultipliers[speedComboBox.SelectedIndex]);
            Debug.WriteLine("timerSimulation Interval " + timerSimulation.Interval + ", " + speedMultipliers[speedComboBox.SelectedIndex]);
        }


        private void zoomComboBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            daGridScrollPosition = new Point(
                (int)Math.Round(daGridViewCenter.X - (DaGrid.Width / 2 * zoomMultipliers[zoomComboBox.SelectedIndex, 1])),
                (int)Math.Round(daGridViewCenter.Y - (DaGrid.Height / 2 * zoomMultipliers[zoomComboBox.SelectedIndex, 1])));

            UpdateDaGridClippingRect();
            DaGrid.Invalidate();
        }

        private void buttonTLightTemp_Click(object sender, EventArgs e)
        {
            switch (nc.segments.Find(x => x.Id == 3768).endNode.tLight.trafficLightState)
            {
                case TrafficLight.State.GREEN:
                    nc.segments.Find(x => x.Id == 3768).endNode.tLight.SwitchToRed();
                    break;
                case TrafficLight.State.RED:
                    nc.segments.Find(x => x.Id == 3768).endNode.tLight.SwitchToGreen();
                    break;
            }

            Invalidate(InvalidationLevel.MAIN_CANVAS_AND_TIMELINE);
        }

        private void manuallyAddRoute()
        {
            this._route = new List<RoadSegment>();//Dayeuhkolot-Buahbatu 
            this._route2 = new List<RoadSegment>();//Buahbatu-Samsat
            this._route3 = new List<RoadSegment>();//Buahbatu - Dayeuhkolot
            this._route4 = new List<RoadSegment>();//Samsat - Buahbatu

            // Route 1 : Dayeuhkolot-Buahbatu
            for (int i = 37366; i <= 37367; i++)
            {
                this._route.Add(nc.segments.Find(x => x.Id == i));
            }
            for (int i = 25065; i < 25068; i++)
            {
                this._route.Add(nc.segments.Find(x => x.Id == i));
            }
            nc.segments.Find(x => x.Id == 37368).endNode.tLight = new TrafficLight();

            // Route 2 : Buahbatu-Samsat
            for (int i = 25374; i < 25377; i++)
            {
                this._route2.Add(nc.segments.Find(x => x.Id == i));
            }
            for (int i = 25071; i < 25074; i++)
            {
                this._route2.Add(nc.segments.Find(x => x.Id == i));
            }

            // Route 3 : Bubat - Dayehkolot
            for (int i = 34267; i <= 34269; i++)
            {
                this._route3.Add(nc.segments.Find(x => x.Id == i));
            }
            for (int i = 25079; i < 25080; i++)
            {
                this._route3.Add(nc.segments.Find(x => x.Id == i));
            }

            //route 4 : Samsat daria arah barat
            for (int i = 33890; i < 33891; i++)
            {
                this._route4.Add(nc.segments.Find(x => x.Id == i));
            }
            for (int i = 27062; i < 27063; i++)
            {
                this._route4.Add(nc.segments.Find(x => x.Id == i));
            }
        }

        private async Task GenerateVehicleFromDb()
        {
            try
            {

                Debug.WriteLine($"[{DateTime.Now}] Fetching data from MongoDB...");
                List<AtcsResult> results = await _mongoService.GetAsync();

                if (results.Count == 0)
                {
                    return;
                }

                _queuedVehiclesRoute1.Clear();
                _queuedVehiclesRoute2.Clear();
                _queuedVehiclesRoute3.Clear();
                _queuedVehiclesRoute4.Clear();

                if (nc != null && nc.Segments != null)
                {
                    foreach (var segment in nc.Segments)
                    {
                        if (segment?.lanes == null) continue;

                        foreach (var lane in segment.lanes)
                        {
                            if (lane?.vehicles == null) continue;

                            foreach (var vehicle in lane.vehicles)
                            {
                                if (vehicle != null && vehicle.IsQueued)
                                {
                                    if (vehicle.StopSegmentId == _stopSegmentId1 && vehicle.currentSegment?.Id == _stopSegmentId1)
                                    {
                                        if (!_queuedVehiclesRoute1.Contains(vehicle)) _queuedVehiclesRoute1.Add(vehicle);
                                    }
                                    else if (vehicle.StopSegmentId == _stopSegmentId2 && vehicle.currentSegment?.Id == _stopSegmentId2)
                                    {
                                        if (!_queuedVehiclesRoute2.Contains(vehicle)) _queuedVehiclesRoute2.Add(vehicle);
                                    }
                                    else if (vehicle.StopSegmentId == _stopSegmentId3 && vehicle.currentSegment?.Id == _stopSegmentId3)
                                    {
                                        if (!_queuedVehiclesRoute3.Contains(vehicle)) _queuedVehiclesRoute3.Add(vehicle);
                                    }
                                    else if (vehicle.StopSegmentId == _stopSegmentId4 && vehicle.currentSegment?.Id == _stopSegmentId4)
                                    {
                                        if (!_queuedVehiclesRoute4.Contains(vehicle)) _queuedVehiclesRoute4.Add(vehicle);
                                    }
                                }
                            }
                        }
                    }
                }

                foreach (AtcsResult result in results)
                {
                    List<RoadSegment> targetRoute = null;
                    long stopId = 0;
                    List<IVehicle> actualQueuedList = null;
                    Func<int, long, IVehicle> vehicleFactory = null;

                    switch (result.CameraId)
                    {
                        case "1001":
                            _queue_route4 = result.QueueCount;
                            targetRoute = _route4;
                            stopId = _stopSegmentId4;
                            actualQueuedList = _queuedVehiclesRoute4;
                            break;
                        case "1401":
                            _queue_route1 = result.QueueCount;
                            targetRoute = _route;
                            stopId = _stopSegmentId1;
                            actualQueuedList = _queuedVehiclesRoute1;
                            break;
                        case "1501":
                            _queue_route2 = result.QueueCount;
                            targetRoute = _route2;
                            stopId = _stopSegmentId2;
                            actualQueuedList = _queuedVehiclesRoute2;
                            break;
                        case "1601":
                            _queue_route3 = result.QueueCount;
                            targetRoute = _route3;
                            stopId = _stopSegmentId3;
                            actualQueuedList = _queuedVehiclesRoute3;
                            break;
                        default:
                            Debug.WriteLine($"Unknown CameraId: {result.CameraId}");
                            continue;
                    }

                    if (targetRoute == null || targetRoute.Count == 0 || stopId == 0 || actualQueuedList == null)
                    {
                        Debug.WriteLine($"Route/Stop/QueueList untuk CameraId {result.CameraId} tidak terkonfigurasi");
                        continue;
                    }

                    int targetQueueCount = result.QueueCount;
                    int currentActualQueueSize = actualQueuedList.Count;

                    if (targetQueueCount > currentActualQueueSize)
                    {
                        int vehiclesToSpawn = targetQueueCount - currentActualQueueSize;

                        if (nc.ActiveVehicleCount + vehiclesToSpawn > MaxActiveVehicles)
                        {
                            vehiclesToSpawn = MaxActiveVehicles - nc.ActiveVehicleCount;
                            if (vehiclesToSpawn < 0) vehiclesToSpawn = 0;
                            Debug.WriteLine($"Spawning dibatasi {vehiclesToSpawn} oleh limit global.");
                        }


                        if (vehiclesToSpawn > 0)
                        {
                            vehicleFactory = (laneIdx, sId) => {
                                int vehType = rnd.Next(0, 4);
                                switch (vehType)
                                {
                                    case 0: return new Car(targetRoute[0], laneIdx, new List<RoadSegment>(targetRoute), sId);
                                    case 1: return new Bus(targetRoute[0], laneIdx, new List<RoadSegment>(targetRoute), sId);
                                    case 2: return new Truck(targetRoute[0], laneIdx, new List<RoadSegment>(targetRoute), sId);
                                    default: return new MotorCycle(targetRoute[0], laneIdx, new List<RoadSegment>(targetRoute), sId);
                                }
                            };
                            Debug.WriteLine($"[{DateTime.Now}] Spawning {vehiclesToSpawn} for route {result.CameraId}.");
                            GenerateVehiclesOfType(vehiclesToSpawn, targetRoute, stopId, vehicleFactory);
                        }
                    }
                    else if (targetQueueCount < currentActualQueueSize)
                    {
                        int vehiclesToRelease = currentActualQueueSize - targetQueueCount;
                        Debug.WriteLine($"[{DateTime.Now}] Releasing {vehiclesToRelease} from route {result.CameraId}.");

                        for (int i = 0; i < vehiclesToRelease; i++)
                        {
                            if (actualQueuedList.Count > 0)
                            {
                                var vehicleToRelease = actualQueuedList[0];
                                if (vehicleToRelease != null && vehicleToRelease.IsQueued)
                                {
                                    vehicleToRelease.IsQueued = false;
                                    vehicleToRelease.IsReleased = true;
                                    Debug.WriteLine($" - Releasing vehicle {vehicleToRelease.GetHashCode()}");
                                    actualQueuedList.RemoveAt(0);
                                }
                                else
                                {
                                    Debug.WriteLine($" - Warning: Vehicle {vehicleToRelease?.GetHashCode()} in queue list was invalid/not queued. Removing.");
                                    actualQueuedList.RemoveAt(0);
                                    i--;
                                }
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                }

                Debug.WriteLine($"[{DateTime.Now}] Tick End. Total active vehicles: {nc.ActiveVehicleCount}");
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"[{DateTime.Now}] Error in GenerateVehicleFromDb: {ex.Message}");
                Debug.WriteLine($"Stack trace: {ex.StackTrace}");
            }
        }

        private List<IVehicle> GenerateVehiclesOfType(int count, List<RoadSegment> roadSegment, long stopSegmentId, Func<int, long, IVehicle> vehicleFactory)
        {
            List<IVehicle> createdVehicles = new List<IVehicle>();
            try
            {
                if (roadSegment == null || roadSegment.Count == 0) return createdVehicles;

                var firstSegment = roadSegment[0];
                if (firstSegment.lanes == null || firstSegment.lanes.Count == 0) return createdVehicles;

                int maxVehiclesPerLane = 5;

                for (int i = 0; i < count; i++)
                {
                    if (nc.ActiveVehicleCount >= MaxActiveVehicles)
                    {
                        Debug.WriteLine($"Global vehicle limit reached ({nc.ActiveVehicleCount}/{MaxActiveVehicles}), stop spawning.");
                        break;
                    }

                    int laneidx = rnd.Next(0, firstSegment.lanes.Count);

                    if (firstSegment.lanes[laneidx].vehicles == null)
                        firstSegment.lanes[laneidx].vehicles = new List<IVehicle>();

                    if (firstSegment.lanes[laneidx].vehicles.Count >= maxVehiclesPerLane)
                    {
                        Debug.WriteLine($"Lane {laneidx} sudah penuh, skip kendaraan");
                        continue;
                    }

                    IVehicle v = vehicleFactory(laneidx, stopSegmentId);

                    firstSegment.lanes[laneidx].vehicles.Add(v);
                    createdVehicles.Add(v);
                }
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"Error in GenerateVehiclesOfType: {ex.Message}");
                Debug.WriteLine(ex.StackTrace);
            }
            return createdVehicles;
        }

        //private void GenerateVehicles()
        //{
        //    #region tempVehGenerate
        //    if ((timeMod % 36) == 0.0)
        //    {
        //        if ((vehCount % 2) == 0)
        //        {
        //            int laneidx = rnd.Next(0, _route[0].lanes.Count);
        //            int vehType = rnd.Next(0, 3);
        //            IVehicle v = null;

        //            if (vehType == 0)
        //            {
        //                v = new Car(_route[0], laneidx, _route);
        //            }
        //            else if (vehType == 1)
        //            {
        //                v = new Bus(_route[0], laneidx, _route);
        //            }
        //            else if (vehType == 2)
        //            {
        //                v = new MotorCycle(this._route[0], laneidx, this._route);
        //            }
        //            else
        //            {
        //                v = new Truck(_route[0], laneidx, _route);
        //            }

        //            _route[0].lanes[laneidx].vehicles.Add(v);
        //            activeVehicles++;

        //            laneidx = rnd.Next(0, _route2[0].lanes.Count);
        //            vehType = rnd.Next(0, 3);

        //            if (vehType == 0)
        //            {
        //                v = new Car(_route2[0], laneidx, _route2);
        //            }
        //            else if (vehType == 1)
        //            {
        //                v = new Bus(_route2[0], laneidx, _route2);
        //            }
        //            else if (vehType == 2)
        //            {
        //                v = new MotorCycle(this._route2[0], laneidx, this._route2);
        //            }
        //            else
        //            {
        //                v = new Truck(_route2[0], laneidx, _route2);
        //            }

        //            _route2[0].lanes[laneidx].vehicles.Add(v);
        //            activeVehicles++;
        //        }
        //        else
        //        {
        //            int laneidx = rnd.Next(0, this._route3[0].lanes.Count);
        //            int vehType = rnd.Next(0, 3);
        //            IVehicle v = null;

        //            if (vehType == 0)
        //            {
        //                v = new Car(this._route3[0], laneidx, this._route3);
        //            }
        //            else if (vehType == 1)
        //            {
        //                v = new Bus(this._route3[0], laneidx, this._route3);
        //            }
        //            else if (vehType == 2)
        //            {
        //                v = new MotorCycle(this._route3[0], laneidx, this._route3);
        //            }
        //            else
        //            {
        //                v = new Truck(this._route3[0], laneidx, this._route3);
        //            }

        //            this._route3[0].lanes[laneidx].vehicles.Add(v);
        //            activeVehicles++;

        //            laneidx = rnd.Next(0, this._route4[0].lanes.Count);
        //            vehType = rnd.Next(0, 3);

        //            if (vehType == 0)
        //            {
        //                v = new Car(this._route4[0], laneidx, this._route4);
        //            }
        //            else if (vehType == 1)
        //            {
        //                v = new Bus(this._route4[0], laneidx, this._route4);
        //            }
        //            else if (vehType == 2)
        //            {
        //                v = new MotorCycle(this._route4[0], laneidx, this._route4);
        //            }
        //            else
        //            {
        //                v = new Truck(this._route4[0], laneidx, this._route4);
        //            }

        //            this._route4[0].lanes[laneidx].vehicles.Add(v);
        //            activeVehicles++;
        //        }
        //        vehCount++;
        //    }
        //    timeMod++;
        //    #endregion
        //}
    }
}