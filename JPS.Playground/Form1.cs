using System.Text.Json;
using System.Text.Json.Serialization;
using JPS.Controls;
using JPS.Data;
using JPS.Models;

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
            SelectMode(EditMode.BrushObstacle);
        }

        // 按系统语言设置工具栏按钮、提示、标题与状态栏文案（中/英）。
        private void ApplyLocalization()
        {
            Text = Loc.T("JPS / A* 寻路测试", "JPS / A* Pathfinding Test");

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
