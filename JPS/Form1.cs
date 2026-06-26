using System.Text.Json;
using System.Text.Json.Serialization;
using JPS.Controls;
using JPS.Models;

namespace JPS
{
    public partial class Form1 : Form
    {
        private static readonly (Func<Color> Color, string Label)[] LegendItems =
        [
            (() => GridControl.ObstacleColor, "阻挡"),
            (() => GridControl.StartColor, "起点 S"),
            (() => GridControl.EndColor, "终点 G"),
            (() => GridControl.PathColor, "路径"),
            (() => GridControl.ExpandedColor, "已扩展"),
            (() => GridControl.FrontierColor, "已入队未扩展"),
            (() => GridControl.ScannedColor, "扫描跳过"),
        ];

        public Form1()
        {
            InitializeComponent();
            SelectMode(EditMode.BrushObstacle);
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

        private void BtnPrecompute_Click(object? sender, EventArgs e) => gridControl.RebuildJumpCache();

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
                Filter = "JSON 地图|*.json",
                FileName = "map.json"
            };

            if (dlg.ShowDialog(this) != DialogResult.OK)
                return;

            try
            {
                var data = gridControl.Export();
                File.WriteAllText(dlg.FileName, JsonSerializer.Serialize(data, JsonOptions));
                statusLabel.Text = $"已保存到 {dlg.FileName}（阻挡 {data.Obstacles.Count} 格）";
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, "保存失败", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void BtnLoad_Click(object? sender, EventArgs e)
        {
            using var dlg = new OpenFileDialog
            {
                Filter = "JSON 地图|*.json"
            };

            if (dlg.ShowDialog(this) != DialogResult.OK)
                return;

            try
            {
                var data = JsonSerializer.Deserialize<MapData>(File.ReadAllText(dlg.FileName));
                if (data == null)
                {
                    MessageBox.Show(this, "文件内容为空或格式不正确。", "载入失败", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }

                gridControl.Import(data);
                statusLabel.Text = $"已载入 {dlg.FileName}（阻挡 {data.Obstacles.Count} 格，原始尺寸 {data.Width}x{data.Height}）";
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, "载入失败", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void GridControl_StatusChanged(object? sender, string message) =>
            statusLabel.Text = message;
    }
}
