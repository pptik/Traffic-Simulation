using System;

namespace SimTMDG
{
    partial class Main
    {
        private System.ComponentModel.IContainer components = null;

        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.tableLayoutPanel1 = new System.Windows.Forms.TableLayoutPanel();
            this.flowLayoutPanel1 = new System.Windows.Forms.FlowLayoutPanel();
            this.speedComboBox = new System.Windows.Forms.ComboBox();
            this.zoomComboBox = new System.Windows.Forms.ComboBox();
            this.playButton = new System.Windows.Forms.Button();
            this.stepButton = new System.Windows.Forms.Button();
            this.tempLoadButton = new System.Windows.Forms.Button();
            this.DaGrid = new SimTMDG.MainForm.CustUserControl();
            this.buttonTLightTemp = new System.Windows.Forms.Button();
            this.timerSimulation = new System.Windows.Forms.Timer(this.components);
            this.tableLayoutPanel1.SuspendLayout();
            this.flowLayoutPanel1.SuspendLayout();
            this.SuspendLayout();
            // 
            // tableLayoutPanel1
            // 
            this.tableLayoutPanel1.ColumnCount = 2;
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 293F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle());
            this.tableLayoutPanel1.Controls.Add(this.flowLayoutPanel1, 1, 0);
            this.tableLayoutPanel1.Controls.Add(this.DaGrid, 1, 1);
            this.tableLayoutPanel1.Controls.Add(this.buttonTLightTemp, 0, 1);
            this.tableLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel1.Location = new System.Drawing.Point(0, 0);
            this.tableLayoutPanel1.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.tableLayoutPanel1.Name = "tableLayoutPanel1";
            this.tableLayoutPanel1.RowCount = 2;
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 44F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 25F));
            this.tableLayoutPanel1.Size = new System.Drawing.Size(1453, 801);
            this.tableLayoutPanel1.TabIndex = 0;
            this.tableLayoutPanel1.Paint += new System.Windows.Forms.PaintEventHandler(this.tableLayoutPanel1_Paint);
            // 
            // flowLayoutPanel1
            // 
            this.flowLayoutPanel1.Controls.Add(this.speedComboBox);
            this.flowLayoutPanel1.Controls.Add(this.zoomComboBox);
            this.flowLayoutPanel1.Controls.Add(this.playButton);
            this.flowLayoutPanel1.Controls.Add(this.stepButton);
            this.flowLayoutPanel1.Controls.Add(this.tempLoadButton);
            this.flowLayoutPanel1.Location = new System.Drawing.Point(297, 4);
            this.flowLayoutPanel1.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.flowLayoutPanel1.Name = "flowLayoutPanel1";
            this.flowLayoutPanel1.Size = new System.Drawing.Size(1156, 36);
            this.flowLayoutPanel1.TabIndex = 0;
            // 
            // speedComboBox
            // 
            this.speedComboBox.FormattingEnabled = true;
            this.speedComboBox.Items.AddRange(new object[] {
            "1x",
            "2x",
            "4x",
            "8x",
            "16x"});
            this.speedComboBox.Location = new System.Drawing.Point(4, 4);
            this.speedComboBox.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.speedComboBox.Name = "speedComboBox";
            this.speedComboBox.Size = new System.Drawing.Size(160, 24);
            this.speedComboBox.TabIndex = 2;
            this.speedComboBox.SelectedIndexChanged += new System.EventHandler(this.speedComboBox_SelectedIndexChanged);
            // 
            // zoomComboBox
            // 
            this.zoomComboBox.FormattingEnabled = true;
            this.zoomComboBox.Items.AddRange(new object[] {
            "5%",
            "10%",
            "15%",
            "20%",
            "25%",
            "33%",
            "50%",
            "67%",
            "100%",
            "150%",
            "200%",
            "400%"});
            this.zoomComboBox.Location = new System.Drawing.Point(172, 4);
            this.zoomComboBox.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.zoomComboBox.Name = "zoomComboBox";
            this.zoomComboBox.Size = new System.Drawing.Size(160, 24);
            this.zoomComboBox.TabIndex = 3;
            this.zoomComboBox.SelectedIndexChanged += new System.EventHandler(this.zoomComboBox_SelectedIndexChanged);
            // 
            // playButton
            // 
            this.playButton.Location = new System.Drawing.Point(340, 4);
            this.playButton.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.playButton.Name = "playButton";
            this.playButton.Size = new System.Drawing.Size(100, 28);
            this.playButton.TabIndex = 0;
            this.playButton.Text = "Play";
            this.playButton.UseVisualStyleBackColor = true;
            this.playButton.Click += new System.EventHandler(this.playButton_Click);
            // 
            // stepButton
            // 
            this.stepButton.Location = new System.Drawing.Point(448, 4);
            this.stepButton.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.stepButton.Name = "stepButton";
            this.stepButton.Size = new System.Drawing.Size(100, 28);
            this.stepButton.TabIndex = 4;
            this.stepButton.Text = "Single Step";
            this.stepButton.UseVisualStyleBackColor = true;
            this.stepButton.Click += new System.EventHandler(this.stepButton_Click);
            // 
            // tempLoadButton
            // 
            this.tempLoadButton.Location = new System.Drawing.Point(556, 4);
            this.tempLoadButton.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.tempLoadButton.Name = "tempLoadButton";
            this.tempLoadButton.Size = new System.Drawing.Size(100, 28);
            this.tempLoadButton.TabIndex = 5;
            this.tempLoadButton.Text = "Load";
            this.tempLoadButton.UseVisualStyleBackColor = true;
            this.tempLoadButton.Click += new System.EventHandler(this.tempLoadButton_Click);
            // 
            // DaGrid
            // 
            this.DaGrid.AutoSizeMode = System.Windows.Forms.AutoSizeMode.GrowAndShrink;
            this.DaGrid.BackColor = System.Drawing.SystemColors.ButtonFace;
            this.DaGrid.CellHeight = 0;
            this.DaGrid.CellSize = new System.Drawing.Size(0, 0);
            this.DaGrid.CellWidth = 0;
            this.DaGrid.Dimension = new System.Drawing.Point(64, 64);
            this.DaGrid.DrawGrid = false;
            this.DaGrid.Location = new System.Drawing.Point(298, 49);
            this.DaGrid.Margin = new System.Windows.Forms.Padding(5, 5, 5, 5);
            this.DaGrid.Max_X = 64;
            this.DaGrid.Max_Y = 64;
            this.DaGrid.Name = "DaGrid";
            this.DaGrid.Size = new System.Drawing.Size(320, 295);
            this.DaGrid.TabIndex = 3;
            this.DaGrid.Paint += new System.Windows.Forms.PaintEventHandler(this.DaGrid_Paint);
            this.DaGrid.MouseDown += new System.Windows.Forms.MouseEventHandler(this.DaGrid_MouseDown);
            this.DaGrid.MouseMove += new System.Windows.Forms.MouseEventHandler(this.DaGrid_MouseMove);
            this.DaGrid.MouseUp += new System.Windows.Forms.MouseEventHandler(this.DaGrid_MouseUp);
            this.DaGrid.MouseWheel += new System.Windows.Forms.MouseEventHandler(this.DaGrid_MouseWheel);
            this.DaGrid.Resize += new System.EventHandler(this.DaGrid_Resize);
            // 
            // buttonTLightTemp
            // 
            this.buttonTLightTemp.Location = new System.Drawing.Point(4, 48);
            this.buttonTLightTemp.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.buttonTLightTemp.Name = "buttonTLightTemp";
            this.buttonTLightTemp.Size = new System.Drawing.Size(285, 28);
            this.buttonTLightTemp.TabIndex = 4;
            this.buttonTLightTemp.Text = "Change Traffic Light";
            this.buttonTLightTemp.UseVisualStyleBackColor = true;
            this.buttonTLightTemp.Click += new System.EventHandler(this.buttonTLightTemp_Click);
            // 
            // timerSimulation
            // 
            this.timerSimulation.Interval = 67;
            this.timerSimulation.Tick += new System.EventHandler(this.timerSimulation_Tick);
            // 
            // Main
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 16F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1453, 801);
            this.Controls.Add(this.tableLayoutPanel1);
            this.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.Name = "Main";
            this.Text = "Simulasi Mikro Lalu Lintas";
            this.tableLayoutPanel1.ResumeLayout(false);
            this.flowLayoutPanel1.ResumeLayout(false);
            this.ResumeLayout(false);
        }
        #endregion

        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel1;
        private System.Windows.Forms.FlowLayoutPanel flowLayoutPanel1;
        private System.Windows.Forms.ComboBox speedComboBox;
        private System.Windows.Forms.ComboBox zoomComboBox;
        private System.Windows.Forms.Button playButton;
        private System.Windows.Forms.Button stepButton;
        private System.Windows.Forms.Timer timerSimulation;
        private MainForm.CustUserControl DaGrid;
        private System.Windows.Forms.Button tempLoadButton;
        private System.Windows.Forms.Button buttonTLightTemp;
    }
}

