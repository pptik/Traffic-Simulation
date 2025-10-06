using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

using SimTMDG.Tools;

namespace SimTMDG.MainForm
{
    public partial class CustUserControl : UserControl
    {
        #region Variable & Properties
        /// <summary>
		/// Grids size
		/// </summary>
        private Point m_Dimension;
        /// <summary>
        /// Size of single cell
        /// </summary>
        private Size m_CellSize;
        /// <summary>
        /// Flags whether the grid should be drawn
        /// </summary>
        private bool m_DrawGrid;

        /// <summary>
        /// Grids size
        /// </summary>
        public Point Dimension
        {
            get { return m_Dimension; }
            set
            {
                m_Dimension = value;
                Invalidate();
            }
        }

        /// <summary>
        /// Grids width
        /// </summary>
        public int Max_X
        {
            get { return m_Dimension.X; }
            set
            {
                m_Dimension.X = value;
                Invalidate();
            }
        }
        /// <summary>
        /// Grids Height
        /// </summary>
        public int Max_Y
        {
            get { return m_Dimension.Y; }
            set
            {
                m_Dimension.Y = value;
                Invalidate();
            }
        }

        /// <summary>
        /// Dimensions of single cell
        /// </summary>
        public Size CellSize
        {
            get { return m_CellSize; }
            set
            {
                m_CellSize = value;
                Invalidate();
            }
        }
        /// <summary>
        /// Width of single cell
        /// </summary>
        public int CellWidth
        {
            get { return m_CellSize.Width; }
            set
            {
                m_CellSize.Width = value;
                Invalidate();
            }
        }
        /// <summary>
        /// Height of single cell
        /// </summary>
        public int CellHeight
        {
            get { return m_CellSize.Height; }
            set
            {
                m_CellSize.Height = value;
                Invalidate();
            }
        }

        /// <summary>
        /// Flags whether the grid should be drawn
        /// </summary>
        public bool DrawGrid
        {
            get { return m_DrawGrid; }
            set { m_DrawGrid = value; }
        }
        #endregion

        public CustUserControl()
        {
            InitializeComponent();

            this.SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.OptimizedDoubleBuffer, true);
            this.DoubleBuffered = true;

            if (DesignMode)
            {
                this.Dimension = new Point(10, 10);
                this.CellSize = new Size(16, 16);
            }
        }

        /// <Summary>
        /// Directed the point client position to the grid
        /// </ Summary>
        /// <Param name = "client position"> to be aligned point </ param>
        /// <Param name = "DockToGrid"> Flag, whether to ever aligned </ param>
        /// <Returns> If DockToGrid == true: the point on the grid of the client position is closest, otherwise client position </ returns>
        public Vector2 DockToGrid(Vector2 clientPosition, bool DockToGrid)
        {
            if (DockToGrid)
            {
                Vector2 toreturn = new Vector2(
                                        (float)Math.Floor(clientPosition.X / this.CellWidth) * this.CellWidth,
                                        (float)Math.Floor(clientPosition.Y / this.CellHeight) * this.CellHeight
                                   );
                return toreturn;
            }else
            {
                return clientPosition;
            }
        }

        #region Event-Handler
        /// <Summary>
        /// Paint method. If DrawGrid == true a computing box grid is drawn here
        /// </ Summary>
        /// <Param name = "sender"> Caller object </ param>
        /// <Param name = "e"> Paint Event arguments </ param>
        private void CustUserControl_Paint(object sender, PaintEventArgs e)
        {
            e.Graphics.Clear(Color.White);
        }
        #endregion
    }
}
