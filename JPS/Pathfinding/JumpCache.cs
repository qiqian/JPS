namespace JPS.Pathfinding;

/// <summary>
/// 预计算的 8 方向跳点距离表（JPS+）。每个格子、每个方向存一个带符号距离：
///   &gt; 0 : 该方向 d 步处有一个跳点；
///   &lt;= 0: 该方向没有跳点，|值| = 撞墙/越界前可前进的格子数。
/// </summary>
public sealed class JumpCache
{
    private readonly int _width;
    private readonly int[] _dist;

    public JumpCache(int width, int height)
    {
        _width = width;
        _dist = new int[width * height * JpsDirections.Count];
    }

    public int Get(int x, int y, int direction) =>
        _dist[(y * _width + x) * JpsDirections.Count + direction];

    public void Set(int x, int y, int direction, int value) =>
        _dist[(y * _width + x) * JpsDirections.Count + direction] = value;
}
