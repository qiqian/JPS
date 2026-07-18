/*
 * Form1.Designer.cs
 * JPS Pathfinding
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

namespace JPS
{
    partial class Form1
    {
        private System.ComponentModel.IContainer components = null;
        private ToolStrip toolStrip1;
        private ToolStripButton btnBrush;
        private ToolStripButton btnStart;
        private ToolStripButton btnEnd;
        private ToolStripButton btnClear;
        private ToolStripButton btnDynamic;
        private ToolStripButton btnStatic;
        private ToolStripButton btnFindPath;
        private ToolStripButton btnFindPathNearest;
        private ToolStripButton btnFindPathAStar;
        private ToolStripLabel bodySizeLabel;
        private ToolStripTextBox txtBodySize;
        private ToolStripButton btnSave;
        private ToolStripButton btnLoad;
        private ToolStripButton btnOpenMap;
        private ToolStripSeparator toolStripSeparator1;
        private ToolStripSeparator toolStripSeparator2;
        private StatusStrip statusStrip1;
        private ToolStripStatusLabel statusLabel;
        private Panel scrollPanel;
        private Panel legendPanel;
        private Controls.GridControl gridControl;

        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        private void InitializeComponent()
        {
            components = new System.ComponentModel.Container();
            toolStrip1 = new ToolStrip();
            btnBrush = new ToolStripButton();
            btnStart = new ToolStripButton();
            btnEnd = new ToolStripButton();
            btnClear = new ToolStripButton();
            btnDynamic = new ToolStripButton();
            btnStatic = new ToolStripButton();
            toolStripSeparator1 = new ToolStripSeparator();
            btnFindPath = new ToolStripButton();
            btnFindPathNearest = new ToolStripButton();
            btnFindPathAStar = new ToolStripButton();
            bodySizeLabel = new ToolStripLabel();
            txtBodySize = new ToolStripTextBox();
            toolStripSeparator2 = new ToolStripSeparator();
            btnSave = new ToolStripButton();
            btnLoad = new ToolStripButton();
            btnOpenMap = new ToolStripButton();
            statusStrip1 = new StatusStrip();
            statusLabel = new ToolStripStatusLabel();
            scrollPanel = new Panel();
            legendPanel = new Panel();
            gridControl = new Controls.GridControl(28);
            toolStrip1.SuspendLayout();
            statusStrip1.SuspendLayout();
            scrollPanel.SuspendLayout();
            SuspendLayout();
            // 
            // toolStrip1
            // 
            toolStrip1.GripStyle = ToolStripGripStyle.Hidden;
            toolStrip1.ImageScalingSize = new Size(20, 20);
            toolStrip1.Items.AddRange(new ToolStripItem[]
            {
                btnBrush, btnStart, btnEnd, btnClear, btnDynamic, btnStatic, toolStripSeparator1, btnFindPath, btnFindPathNearest, btnFindPathAStar,
                bodySizeLabel, txtBodySize, toolStripSeparator2, btnSave, btnLoad, btnOpenMap
            });
            toolStrip1.Location = new Point(0, 0);
            toolStrip1.Name = "toolStrip1";
            toolStrip1.Size = new Size(1284, 31);
            toolStrip1.TabIndex = 0;
            // 
            // btnBrush
            // 
            btnBrush.CheckOnClick = true;
            btnBrush.DisplayStyle = ToolStripItemDisplayStyle.Text;
            btnBrush.Name = "btnBrush";
            btnBrush.Size = new Size(60, 28);
            btnBrush.Text = "刷阻挡";
            btnBrush.ToolTipText = "点空地刷 2×2 阻挡，点阻挡清除 1 格";
            btnBrush.Click += BtnBrush_Click;
            // 
            // btnStart
            // 
            btnStart.CheckOnClick = true;
            btnStart.DisplayStyle = ToolStripItemDisplayStyle.Text;
            btnStart.Name = "btnStart";
            btnStart.Size = new Size(60, 28);
            btnStart.Text = "起点";
            btnStart.Click += BtnStart_Click;
            // 
            // btnEnd
            // 
            btnEnd.CheckOnClick = true;
            btnEnd.DisplayStyle = ToolStripItemDisplayStyle.Text;
            btnEnd.Name = "btnEnd";
            btnEnd.Size = new Size(60, 28);
            btnEnd.Text = "终点";
            btnEnd.Click += BtnEnd_Click;
            // 
            // btnClear
            // 
            btnClear.DisplayStyle = ToolStripItemDisplayStyle.Text;
            btnClear.Name = "btnClear";
            btnClear.Size = new Size(60, 28);
            btnClear.Text = "清除";
            btnClear.Click += BtnClear_Click;
            // 
            // btnDynamic
            // 
            btnDynamic.CheckOnClick = true;
            btnDynamic.DisplayStyle = ToolStripItemDisplayStyle.Text;
            btnDynamic.Name = "btnDynamic";
            btnDynamic.Size = new Size(80, 28);
            btnDynamic.Text = "动态";
            btnDynamic.ToolTipText = "动态障碍测试";
            btnDynamic.Click += BtnDynamic_Click;
            //
            // btnStatic
            //
            btnStatic.CheckOnClick = true;
            btnStatic.DisplayStyle = ToolStripItemDisplayStyle.Text;
            btnStatic.Name = "btnStatic";
            btnStatic.Size = new Size(80, 28);
            btnStatic.Text = "静态";
            btnStatic.ToolTipText = "静态障碍测试";
            btnStatic.Click += BtnStatic_Click;
            //
            // btnFindPath
            // 
            btnFindPath.DisplayStyle = ToolStripItemDisplayStyle.Text;
            btnFindPath.Name = "btnFindPath";
            btnFindPath.Size = new Size(80, 28);
            btnFindPath.Text = "JPS寻路";
            btnFindPath.Click += BtnFindPath_Click;
            //
            // btnFindPathNearest
            //
            btnFindPathNearest.DisplayStyle = ToolStripItemDisplayStyle.Text;
            btnFindPathNearest.Name = "btnFindPathNearest";
            btnFindPathNearest.Size = new Size(80, 28);
            btnFindPathNearest.Text = "JPS就近";
            btnFindPathNearest.ToolTipText = "JPS 就近寻路：终点可落在阻挡上，会 snap 到最近接触格；不可达时停在最近可达点";
            btnFindPathNearest.Click += BtnFindPathNearest_Click;
            //
            // btnFindPathAStar
            // 
            btnFindPathAStar.DisplayStyle = ToolStripItemDisplayStyle.Text;
            btnFindPathAStar.Name = "btnFindPathAStar";
            btnFindPathAStar.Size = new Size(80, 28);
            btnFindPathAStar.Text = "A*寻路";
            btnFindPathAStar.Click += BtnFindPathAStar_Click;
            //
            // bodySizeLabel
            //
            bodySizeLabel.Name = "bodySizeLabel";
            bodySizeLabel.Size = new Size(45, 28);
            bodySizeLabel.Text = "体型:";
            bodySizeLabel.ToolTipText = "单位中心到边缘的格数；adapter 会按此值扩展阻挡";
            //
            // txtBodySize
            //
            txtBodySize.AccessibleName = "体型阔边";
            txtBodySize.MaxLength = 3;
            txtBodySize.Name = "txtBodySize";
            txtBodySize.Size = new Size(42, 31);
            txtBodySize.Text = "0";
            txtBodySize.TextBoxTextAlign = HorizontalAlignment.Center;
            txtBodySize.ToolTipText = "0 表示 1×1；1 表示占用宽度 3 格";
            txtBodySize.KeyDown += BodySize_KeyDown;
            txtBodySize.Leave += BodySize_Leave;
            // 
            // btnSave
            // 
            btnSave.DisplayStyle = ToolStripItemDisplayStyle.Text;
            btnSave.Name = "btnSave";
            btnSave.Size = new Size(60, 28);
            btnSave.Text = "保存";
            btnSave.ToolTipText = "把阻挡、起点、终点保存为 JSON";
            btnSave.Click += BtnSave_Click;
            // 
            // btnLoad
            // 
            btnLoad.DisplayStyle = ToolStripItemDisplayStyle.Text;
            btnLoad.Name = "btnLoad";
            btnLoad.Size = new Size(60, 28);
            btnLoad.Text = "载入";
            btnLoad.ToolTipText = "从 JSON 载入阻挡、起点、终点";
            btnLoad.Click += BtnLoad_Click;
            // 
            // btnOpenMap
            // 
            btnOpenMap.DisplayStyle = ToolStripItemDisplayStyle.Text;
            btnOpenMap.Name = "btnOpenMap";
            btnOpenMap.Size = new Size(80, 28);
            btnOpenMap.Text = "打开地图";
            btnOpenMap.ToolTipText = "打开 MovingAI .map 基准地图";
            btnOpenMap.Click += BtnOpenMap_Click;
            // 
            // statusStrip1
            // 
            statusStrip1.ImageScalingSize = new Size(20, 20);
            statusStrip1.Items.AddRange(new ToolStripItem[] { statusLabel });
            statusStrip1.Location = new Point(0, 839);
            statusStrip1.Name = "statusStrip1";
            statusStrip1.Size = new Size(1284, 26);
            statusStrip1.TabIndex = 1;
            // 
            // statusLabel
            // 
            statusLabel.Name = "statusLabel";
            statusLabel.Size = new Size(174, 20);
            statusLabel.Text = "左键刷阻挡，右键擦除阻挡";
            // 
            // legendPanel
            // 
            legendPanel.BackColor = Color.FromArgb(45, 45, 48);
            legendPanel.Dock = DockStyle.Top;
            legendPanel.Location = new Point(0, 31);
            legendPanel.Name = "legendPanel";
            legendPanel.Size = new Size(1284, 30);
            legendPanel.TabIndex = 3;
            legendPanel.Paint += LegendPanel_Paint;
            // 
            // scrollPanel
            // 
            scrollPanel.AutoScroll = false;   // 滚动由 GridControl 自身处理（定尺地图）
            scrollPanel.Controls.Add(gridControl);
            scrollPanel.Dock = DockStyle.Fill;
            scrollPanel.Location = new Point(0, 61);
            scrollPanel.Name = "scrollPanel";
            scrollPanel.Size = new Size(1284, 778);
            scrollPanel.TabIndex = 2;
            // 
            // gridControl
            // 
            gridControl.Dock = DockStyle.Fill;
            gridControl.Name = "gridControl";
            gridControl.TabIndex = 0;
            gridControl.TabStop = true;
            gridControl.StatusChanged += GridControl_StatusChanged;
            // 
            // Form1
            // 
            AutoScaleDimensions = new SizeF(9F, 20F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(1284, 865);
            Controls.Add(scrollPanel);
            Controls.Add(statusStrip1);
            Controls.Add(legendPanel);
            Controls.Add(toolStrip1);
            MinimumSize = new Size(900, 600);
            Name = "Form1";
            StartPosition = FormStartPosition.CenterScreen;
            WindowState = FormWindowState.Maximized;
            Text = "JPS / A* 寻路测试";
            toolStrip1.ResumeLayout(false);
            toolStrip1.PerformLayout();
            statusStrip1.ResumeLayout(false);
            statusStrip1.PerformLayout();
            scrollPanel.ResumeLayout(false);
            ResumeLayout(false);
            PerformLayout();
        }
    }
}
