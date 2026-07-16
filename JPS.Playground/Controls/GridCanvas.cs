using System.Numerics;
using System.Runtime.Versioning;
using Vortice;
using Vortice.DCommon;
using Vortice.Direct2D1;
using Vortice.DXGI;
using Vortice.Mathematics;
using static Vortice.Direct2D1.D2D1;
using Color = System.Drawing.Color;
using Size = System.Drawing.Size;

namespace JPS.Controls;

internal interface IGridCanvas
{
    void FillRectangle(Color color, float x, float y, float width, float height);
    void DrawRectangle(Color color, float width, float x, float y, float rectWidth, float rectHeight);
    void FillEllipse(Color color, float x, float y, float width, float height);
    void DrawEllipse(Color color, float width, float x, float y, float ellipseWidth, float ellipseHeight);
    void DrawLine(Color color, float width, PointF start, PointF end);
    void DrawLines(Color color, float width, IReadOnlyList<PointF> points);
    void FillPolygon(Color color, IReadOnlyList<PointF> points);
    void DrawPolygon(Color color, float width, IReadOnlyList<PointF> points);
    void DrawBitmap(Bitmap bitmap, Rectangle destination);
    void DrawMarkerGlyph(Color color, string glyph, float x, float y, float size);
}

internal sealed class GdiGridCanvas(Graphics graphics) : IGridCanvas, IDisposable
{
    private readonly Dictionary<int, SolidBrush> _brushes = [];
    private readonly Graphics _graphics = graphics;

    private SolidBrush Brush(Color color)
    {
        if (!_brushes.TryGetValue(color.ToArgb(), out var brush))
            _brushes.Add(color.ToArgb(), brush = new SolidBrush(color));
        return brush;
    }

    public void FillRectangle(Color color, float x, float y, float width, float height) =>
        _graphics.FillRectangle(Brush(color), x, y, width, height);

    public void DrawRectangle(Color color, float width, float x, float y, float rectWidth, float rectHeight)
    {
        using var pen = new Pen(color, width);
        _graphics.DrawRectangle(pen, x, y, rectWidth, rectHeight);
    }

    public void FillEllipse(Color color, float x, float y, float width, float height) =>
        _graphics.FillEllipse(Brush(color), x, y, width, height);

    public void DrawEllipse(Color color, float width, float x, float y, float ellipseWidth, float ellipseHeight)
    {
        using var pen = new Pen(color, width);
        _graphics.DrawEllipse(pen, x, y, ellipseWidth, ellipseHeight);
    }

    public void DrawLine(Color color, float width, PointF start, PointF end)
    {
        using var pen = new Pen(color, width);
        _graphics.DrawLine(pen, start, end);
    }

    public void DrawLines(Color color, float width, IReadOnlyList<PointF> points)
    {
        if (points.Count < 2) return;
        using var pen = new Pen(color, width)
        {
            StartCap = System.Drawing.Drawing2D.LineCap.Round,
            EndCap = System.Drawing.Drawing2D.LineCap.Round,
            LineJoin = System.Drawing.Drawing2D.LineJoin.Round
        };
        _graphics.DrawLines(pen, points.ToArray());
    }

    public void FillPolygon(Color color, IReadOnlyList<PointF> points) =>
        _graphics.FillPolygon(Brush(color), points.ToArray());

    public void DrawPolygon(Color color, float width, IReadOnlyList<PointF> points)
    {
        using var pen = new Pen(color, width);
        _graphics.DrawPolygon(pen, points.ToArray());
    }

    public void DrawBitmap(Bitmap bitmap, Rectangle destination) => _graphics.DrawImage(bitmap, destination);

    public void DrawMarkerGlyph(Color color, string glyph, float x, float y, float size)
    {
        using var font = new Font("Segoe UI", Math.Max(6f, size * 0.45f), FontStyle.Bold);
        var measured = _graphics.MeasureString(glyph, font);
        _graphics.DrawString(glyph, font, Brush(color), x + (size - measured.Width) / 2f, y + (size - measured.Height) / 2f);
    }

    public void Dispose()
    {
        foreach (var brush in _brushes.Values) brush.Dispose();
        _brushes.Clear();
    }
}

/// <summary>Hardware Direct2D HWND target. Direct2D uses Direct3D for hardware rendering.</summary>
[SupportedOSPlatform("windows")]
internal sealed class Direct2DGridCanvas : IGridCanvas, IDisposable
{
    private readonly ID2D1Factory _factory = D2D1CreateFactory<ID2D1Factory>(FactoryType.SingleThreaded);
    private readonly Dictionary<int, ID2D1SolidColorBrush> _brushes = [];
    private readonly Dictionary<Bitmap, ID2D1Bitmap> _bitmaps = [];
    private ID2D1HwndRenderTarget? _target;
    private bool _drawing;

    public bool Begin(nint hwnd, Size clientSize, Color clearColor, Point scrollOffset)
    {
        if (clientSize.Width <= 0 || clientSize.Height <= 0) return false;

        try
        {
            if (_target is null)
            {
                // DPI 强制 96（1 DIP = 1 物理像素）。App 为 SystemAware DPI：ClientSize/鼠标/_cellSize 全是
                // 物理像素，而 D2D 默认 DPI=0 会取桌面 DPI（如 150% 时 144），把按像素画的内容再放大 1.5×——
                // 导致地图缩放错、点击落错格。锁 96 让 D2D 坐标空间与 WinForms 像素空间一致。
                var properties = new RenderTargetProperties(
                    RenderTargetType.Hardware,
                    new PixelFormat(Format.B8G8R8A8_UNorm, Vortice.DCommon.AlphaMode.Ignore),
                    96, 96,
                    RenderTargetUsage.None,
                    FeatureLevel.Default);
                var hwndProperties = new HwndRenderTargetProperties
                {
                    Hwnd = hwnd,
                    PixelSize = new SizeI(clientSize.Width, clientSize.Height),
                    PresentOptions = PresentOptions.None
                };
                _target = _factory.CreateHwndRenderTarget(properties, hwndProperties);
            }
            else if (_target.PixelSize.Width != clientSize.Width || _target.PixelSize.Height != clientSize.Height)
            {
                _target.Resize(new SizeI(clientSize.Width, clientSize.Height));
            }

            _target.BeginDraw();
            _drawing = true;
            _target.Transform = Matrix3x2.Identity;
            _target.Clear(ToColor4(clearColor));
            _target.Transform = Matrix3x2.CreateTranslation(scrollOffset.X, scrollOffset.Y);
            return true;
        }
        catch
        {
            DiscardTarget();
            return false;
        }
    }

    public void End()
    {
        if (!_drawing || _target is null) return;
        _drawing = false;
        try
        {
            var result = _target.EndDraw(out _, out _);
            if (result.Failure) DiscardTarget();
        }
        catch
        {
            DiscardTarget();
        }
    }

    private ID2D1SolidColorBrush Brush(Color color)
    {
        if (_target is null) throw new InvalidOperationException("Direct2D drawing has not begun.");
        if (!_brushes.TryGetValue(color.ToArgb(), out var brush))
            _brushes.Add(color.ToArgb(), brush = _target.CreateSolidColorBrush(ToColor4(color)));
        return brush;
    }

    private static Color4 ToColor4(Color color) => new(color.R / 255f, color.G / 255f, color.B / 255f, color.A / 255f);
    private static RawRectF Rect(float x, float y, float width, float height) => new(x, y, x + width, y + height);
    private static Ellipse Ellipse(float x, float y, float width, float height) =>
        new(new Vector2(x + width / 2f, y + height / 2f), width / 2f, height / 2f);

    public void FillRectangle(Color color, float x, float y, float width, float height) =>
        _target!.FillRectangle(Rect(x, y, width, height), Brush(color));

    public void DrawRectangle(Color color, float width, float x, float y, float rectWidth, float rectHeight) =>
        _target!.DrawRectangle(Rect(x, y, rectWidth, rectHeight), Brush(color), width);

    public void FillEllipse(Color color, float x, float y, float width, float height) =>
        _target!.FillEllipse(Ellipse(x, y, width, height), Brush(color));

    public void DrawEllipse(Color color, float width, float x, float y, float ellipseWidth, float ellipseHeight) =>
        _target!.DrawEllipse(Ellipse(x, y, ellipseWidth, ellipseHeight), Brush(color), width);

    public void DrawLine(Color color, float width, PointF start, PointF end) =>
        _target!.DrawLine(new Vector2(start.X, start.Y), new Vector2(end.X, end.Y), Brush(color), width);

    public void DrawLines(Color color, float width, IReadOnlyList<PointF> points)
    {
        for (int i = 1; i < points.Count; i++) DrawLine(color, width, points[i - 1], points[i]);
        float radius = width / 2f;
        for (int i = 1; i + 1 < points.Count; i++) FillEllipse(color, points[i].X - radius, points[i].Y - radius, width, width);
    }

    public void FillPolygon(Color color, IReadOnlyList<PointF> points)
    {
        if (points.Count < 3) return;
        using var geometry = _factory.CreatePathGeometry();
        using var sink = geometry.Open();
        sink.BeginFigure(new Vector2(points[0].X, points[0].Y), FigureBegin.Filled);
        for (int i = 1; i < points.Count; i++) sink.AddLine(new Vector2(points[i].X, points[i].Y));
        sink.EndFigure(FigureEnd.Closed);
        sink.Close();
        _target!.FillGeometry(geometry, Brush(color));
    }

    public void DrawPolygon(Color color, float width, IReadOnlyList<PointF> points)
    {
        if (points.Count < 2) return;
        for (int i = 0; i < points.Count; i++) DrawLine(color, width, points[i], points[(i + 1) % points.Count]);
    }

    public void DrawBitmap(Bitmap bitmap, Rectangle destination)
    {
        if (!_bitmaps.TryGetValue(bitmap, out var direct2DBitmap))
        {
            var bounds = new Rectangle(0, 0, bitmap.Width, bitmap.Height);
            var data = bitmap.LockBits(bounds, System.Drawing.Imaging.ImageLockMode.ReadOnly, System.Drawing.Imaging.PixelFormat.Format32bppPArgb);
            try
            {
                var properties = new BitmapProperties(
                    new PixelFormat(Format.B8G8R8A8_UNorm, Vortice.DCommon.AlphaMode.Premultiplied));
                direct2DBitmap = _target!.CreateBitmap(
                    new SizeI(bitmap.Width, bitmap.Height), data.Scan0, (uint)data.Stride, properties);
                _bitmaps.Add(bitmap, direct2DBitmap);
            }
            finally
            {
                bitmap.UnlockBits(data);
            }
        }

        _target!.DrawBitmap(
            direct2DBitmap,
            Rect(destination.X, destination.Y, destination.Width, destination.Height),
            1f,
            BitmapInterpolationMode.NearestNeighbor,
            null);
    }

    public void DrawMarkerGlyph(Color color, string glyph, float x, float y, float size)
    {
        float left = x + size * .34f, right = x + size * .66f;
        float top = y + size * .28f, mid = y + size * .50f, bottom = y + size * .72f;
        float stroke = Math.Max(1.2f, size * .09f);
        if (glyph == "S")
        {
            DrawLines(color, stroke, [new(right, top), new(left, top), new(left, mid), new(right, mid), new(right, bottom), new(left, bottom)]);
        }
        else
        {
            DrawLines(color, stroke, [new(right, top), new(left, top), new(left, bottom), new(right, bottom), new(right, mid), new(x + size * .53f, mid)]);
        }
    }

    private void DiscardTarget()
    {
        _drawing = false;
        foreach (var brush in _brushes.Values) brush.Dispose();
        _brushes.Clear();
        foreach (var bitmap in _bitmaps.Values) bitmap.Dispose();
        _bitmaps.Clear();
        _target?.Dispose();
        _target = null;
    }

    public void Dispose()
    {
        DiscardTarget();
        _factory.Dispose();
    }
}
