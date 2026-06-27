using System.Drawing.Drawing2D;
using System.Text.Json;
using System.Text.Json.Serialization;
using JPS.Controls;
using JPS.Data;
using JPS.Models;
using JPS.Pathfinding;

namespace JPS
{
    public partial class Form1 : Form
    {
        private static readonly (Func<Color> Color, string Label)[] LegendItems =
        [
            (() => GridControl.ObstacleColor, Loc.T("阻挡", "Obstacle")),
            (() => GridControl.StartColor, Loc.T("起点 S", "Start S")),
            (() => GridControl.EndColor, Loc.T("终点 G", "Goal G")),
            (() => GridControl.PathColor, Loc.T("路径", "Path")),
            (() => GridControl.SmoothPathColor, Loc.T("平滑路径", "Smoothed")),
            (() => GridControl.ExpandedColor, Loc.T("已扩展", "Expanded")),
            (() => GridControl.FrontierColor, Loc.T("已入队未扩展", "Frontier")),
            (() => GridControl.ScannedColor, Loc.T("扫描跳过", "Scanned")),
            (() => GridControl.JumpFreshColor, Loc.T("本次更新跳点", "New jump")),
            (() => GridControl.JumpCleanColor, Loc.T("已缓存跳点", "Cached jump")),
        ];

        public Form1()
        {
            InitializeComponent();
            ApplyLocalization();
            BuildIcons();
            SelectMode(EditMode.BrushObstacle);
        }

        // 按系统语言设置工具栏按钮、提示、标题与状态栏文案（中/英）。
        private void ApplyLocalization()
        {
            // 标题注明构建配置（斜穿角 / 多线程，由 JPS.Core 编译期常量决定）
            Text = Loc.Zh
                ? $"JPS / A* 寻路测试 — 斜穿角:{(JpsBuildInfo.CornerCutting ? "允许" : "禁止")} | 多线程:{(JpsBuildInfo.ConcurrentCache ? "开" : "关")}"
                : $"JPS / A* Pathfinding Test — corner-cutting:{(JpsBuildInfo.CornerCutting ? "on" : "off")} | multithread:{(JpsBuildInfo.ConcurrentCache ? "on" : "off")}";

            btnBrush.Text = Loc.T("刷阻挡", "Wall");
            btnBrush.ToolTipText = Loc.T("点空地刷 2×2 阻挡，点阻挡清除 1 格",
                "Click empty to paint a 2×2 wall; click a wall to erase 1 cell");
            btnStart.Text = Loc.T("起点", "Start");
            btnEnd.Text = Loc.T("终点", "Goal");
            btnClear.Text = Loc.T("清除", "Clear");
            btnFindPath.Text = Loc.T("JPS寻路", "JPS Path");
            btnFindPathAStar.Text = Loc.T("A*寻路", "A* Path");
            btnSave.Text = Loc.T("保存", "Save");
            btnSave.ToolTipText = Loc.T("把阻挡、起点、终点保存为 JSON",
                "Save obstacles, start and goal to JSON");
            btnLoad.Text = Loc.T("载入", "Load");
            btnLoad.ToolTipText = Loc.T("从 JSON 载入阻挡、起点、终点",
                "Load obstacles, start and goal from JSON");
            btnOpenMap.Text = Loc.T("地图", "Map");
            btnOpenMap.ToolTipText = Loc.T("打开 MovingAI .map 基准地图",
                "Open a MovingAI .map benchmark map");

            statusLabel.Text = Loc.T("左键刷阻挡，右键擦除阻挡",
                "Left-click to paint walls, right-click to erase");
        }

        // 用 GDI+ 现画一组简易位图图标（无外部资源），配色与网格语义一致，显示为“图标 + 文字”。
        private void BuildIcons()
        {
            btnBrush.Image = IconWall();
            btnStart.Image = IconDot(GridControl.StartColor);
            btnEnd.Image = IconDot(GridControl.EndColor);
            btnClear.Image = IconClear();
            btnFindPath.Image = IconJps();
            btnFindPathAStar.Image = IconAStar();
            btnSave.Image = IconArrow(down: true);
            btnLoad.Image = IconArrow(down: false);
            btnOpenMap.Image = IconGrid();

            foreach (var b in new[] { btnBrush, btnStart, btnEnd, btnClear, btnFindPath, btnFindPathAStar, btnSave, btnLoad, btnOpenMap })
                b.DisplayStyle = ToolStripItemDisplayStyle.ImageAndText;
        }

        private static readonly Color IconNeutral = Color.FromArgb(72, 80, 92);
        private static readonly Color IconBlue = Color.FromArgb(70, 120, 190);

        private static Bitmap MakeIcon(Action<Graphics> draw)
        {
            var bmp = new Bitmap(20, 20);
            using var g = Graphics.FromImage(bmp);
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.Clear(Color.Transparent);
            draw(g);
            return bmp;
        }

        // 刷阻挡：2×2 障碍块
        private static Bitmap IconWall() => MakeIcon(g =>
        {
            using var b = new SolidBrush(GridControl.ObstacleColor);
            g.FillRectangle(b, 3, 3, 6, 6);
            g.FillRectangle(b, 11, 3, 6, 6);
            g.FillRectangle(b, 3, 11, 6, 6);
            g.FillRectangle(b, 11, 11, 6, 6);
        });

        // 起点/终点：实心圆 + 白边（复用网格起终点配色）
        private static Bitmap IconDot(Color color) => MakeIcon(g =>
        {
            using var b = new SolidBrush(color);
            using var p = new Pen(Color.White, 1.6f);
            g.FillEllipse(b, 3, 3, 14, 14);
            g.DrawEllipse(p, 3, 3, 14, 14);
        });

        // 清除：红色 ✗
        private static Bitmap IconClear() => MakeIcon(g =>
        {
            using var p = new Pen(Color.FromArgb(220, 70, 70), 3f) { StartCap = LineCap.Round, EndCap = LineCap.Round };
            g.DrawLine(p, 5, 5, 15, 15);
            g.DrawLine(p, 15, 5, 5, 15);
        });

        // JPS 寻路：金色折线 + 箭头（干净跳跃）
        private static Bitmap IconJps() => MakeIcon(g =>
        {
            using var p = new Pen(GridControl.PathColor, 2.4f) { StartCap = LineCap.Round };
            p.CustomEndCap = new AdjustableArrowCap(2.5f, 2.5f);
            g.DrawLines(p, new[] { new PointF(3, 16), new PointF(8, 9), new PointF(12, 13), new PointF(17, 4) });
        });

        // A* 寻路：绿色折线 + 节点点（扩展节点多）
        private static Bitmap IconAStar() => MakeIcon(g =>
        {
            var pts = new[] { new PointF(3, 16), new PointF(8, 9), new PointF(12, 13), new PointF(17, 4) };
            using var p = new Pen(GridControl.ExpandedColor, 2.2f) { StartCap = LineCap.Round, EndCap = LineCap.Round };
            g.DrawLines(p, pts);
            using var b = new SolidBrush(GridControl.ExpandedColor);
            foreach (var pt in pts)
                g.FillEllipse(b, pt.X - 2.2f, pt.Y - 2.2f, 4.4f, 4.4f);
        });

        // 保存/载入：箭头 + 底托盘横线（向下=存，向上=取）
        private static Bitmap IconArrow(bool down) => MakeIcon(g =>
        {
            using var arrow = new Pen(IconBlue, 2.4f) { StartCap = LineCap.Round };
            arrow.CustomEndCap = new AdjustableArrowCap(2.4f, 2.4f);
            if (down) g.DrawLine(arrow, 10, 3, 10, 13);
            else g.DrawLine(arrow, 10, 15, 10, 5);
            using var tray = new Pen(IconBlue, 2.4f) { StartCap = LineCap.Round, EndCap = LineCap.Round };
            g.DrawLine(tray, 4, 17, 16, 17);
        });

        // 打开地图：3×3 网格
        private static Bitmap IconGrid() => MakeIcon(g =>
        {
            using var p = new Pen(IconNeutral, 1.6f);
            g.DrawRectangle(p, 3, 3, 13, 13);
            g.DrawLine(p, 3, 8, 16, 8);
            g.DrawLine(p, 3, 12, 16, 12);
            g.DrawLine(p, 8, 3, 8, 16);
            g.DrawLine(p, 12, 3, 12, 16);
        });

        private void LegendPanel_Paint(object? sender, PaintEventArgs e)
        {
            var g = e.Graphics;
            using var font = new Font("Segoe UI", 9.5f);
            using var textBrush = new SolidBrush(Color.White);
            using var borderPen = new Pen(Color.FromArgb(95, 95, 100));

            const float swatch = 16f;
            float x = 10f;
            float panelH = legendPanel.Height;

            foreach (var (colorFn, label) in LegendItems)
            {
                float sy = (panelH - swatch) / 2f;
                using (var b = new SolidBrush(colorFn()))
                    g.FillRectangle(b, x, sy, swatch, swatch);
                g.DrawRectangle(borderPen, x, sy, swatch, swatch);
                x += swatch + 5f;

                var size = g.MeasureString(label, font);
                g.DrawString(label, font, textBrush, x, (panelH - size.Height) / 2f);
                x += size.Width + 18f;
            }
        }

        private void SelectMode(EditMode mode)
        {
            btnBrush.Checked = mode == EditMode.BrushObstacle;
            btnStart.Checked = mode == EditMode.SetStart;
            btnEnd.Checked = mode == EditMode.SetEnd;
            gridControl.SetMode(mode);
        }

        private void BtnBrush_Click(object? sender, EventArgs e) => SelectMode(EditMode.BrushObstacle);

        private void BtnStart_Click(object? sender, EventArgs e) => SelectMode(EditMode.SetStart);

        private void BtnEnd_Click(object? sender, EventArgs e) => SelectMode(EditMode.SetEnd);

        private void BtnClear_Click(object? sender, EventArgs e) => gridControl.ClearMap();

        private void BtnFindPath_Click(object? sender, EventArgs e) => gridControl.RunJps();

        private void BtnFindPathAStar_Click(object? sender, EventArgs e) => gridControl.RunAStar();

        private static readonly JsonSerializerOptions JsonOptions = new()
        {
            WriteIndented = true,
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull
        };

        private void BtnSave_Click(object? sender, EventArgs e)
        {
            using var dlg = new SaveFileDialog
            {
                Filter = Loc.T("JSON 地图|*.json", "JSON Map|*.json"),
                FileName = "map.json"
            };

            if (dlg.ShowDialog(this) != DialogResult.OK)
                return;

            try
            {
                var data = gridControl.Export();
                File.WriteAllText(dlg.FileName, JsonSerializer.Serialize(data, JsonOptions));
                statusLabel.Text = Loc.Zh
                    ? $"已保存到 {dlg.FileName}（阻挡 {data.Obstacles.Count} 格）"
                    : $"Saved to {dlg.FileName} ({data.Obstacles.Count} walls)";
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, Loc.T("保存失败", "Save failed"), MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void BtnLoad_Click(object? sender, EventArgs e)
        {
            using var dlg = new OpenFileDialog
            {
                Filter = Loc.T("JSON 地图|*.json", "JSON Map|*.json")
            };

            if (dlg.ShowDialog(this) != DialogResult.OK)
                return;

            try
            {
                var data = JsonSerializer.Deserialize<MapData>(File.ReadAllText(dlg.FileName));
                if (data == null)
                {
                    MessageBox.Show(this, Loc.T("文件内容为空或格式不正确。", "File is empty or malformed."),
                        Loc.T("载入失败", "Load failed"), MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }

                gridControl.Import(data);
                statusLabel.Text = Loc.Zh
                    ? $"已载入 {dlg.FileName}（阻挡 {data.Obstacles.Count} 格，原始尺寸 {data.Width}x{data.Height}）"
                    : $"Loaded {dlg.FileName} ({data.Obstacles.Count} walls, original size {data.Width}x{data.Height})";
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, Loc.T("载入失败", "Load failed"), MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void BtnOpenMap_Click(object? sender, EventArgs e)
        {
            using var dlg = new OpenFileDialog
            {
                Filter = Loc.T("MovingAI 地图|*.map", "MovingAI map|*.map")
            };

            if (dlg.ShowDialog(this) != DialogResult.OK)
                return;

            try
            {
                var map = MovingAiMap.Parse(File.ReadAllText(dlg.FileName));
                gridControl.LoadFixedMap(map);
                string name = Path.GetFileName(dlg.FileName);
                statusLabel.Text = Loc.Zh
                    ? $"已打开 {name}（{map.Width}×{map.Height}）。设起点/终点后寻路。"
                    : $"Opened {name} ({map.Width}×{map.Height}). Set start/goal, then run.";
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, Loc.T("打开失败", "Open failed"), MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void GridControl_StatusChanged(object? sender, string message) =>
            statusLabel.Text = message;
    }
}
