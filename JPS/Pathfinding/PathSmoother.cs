using System.Numerics;
using JPS.Models;

namespace JPS.Pathfinding;

/// <summary>
/// 前向增量视线拉直（forward-incremental string pulling）路径平滑。
/// 从当前锚点出发不断向前延伸，直到与某点失去视线，就把上一个可见点定为新锚点。
/// 每个路径点只做一次视线检测，最坏复杂度 O(n·L)，比"从末端找最远点"的贪心(O(n^3))更快，
/// 且视线检测针对整张地图的自由空间，开阔区域能大幅抄近道。
/// 输出为连续格坐标（格中心 = cx+0.5），仅用于显示，不参与整数寻路。
/// </summary>
public static class PathSmoother
{
    public static List<Vector2> Smooth(GridMap map, List<(int X, int Y)> path)
    {
        var result = new List<Vector2>();
        if (path.Count == 0)
            return result;

        result.Add(Center(path[0]));
        if (path.Count == 1)
            return result;

        int anchor = 0;
        for (int i = 2; i < path.Count; i++)
        {
            // 锚点到 path[i] 失去视线：把上一个可见点 path[i-1] 固定为新锚点
            if (!LineOfSight(map, path[anchor].X, path[anchor].Y, path[i].X, path[i].Y))
            {
                result.Add(Center(path[i - 1]));
                anchor = i - 1;
            }
        }

        result.Add(Center(path[^1]));
        return result;
    }

    private static Vector2 Center((int X, int Y) c) => new(c.X + 0.5f, c.Y + 0.5f);

    // 超覆盖(supercover)直线视线检测：遍历线段经过的每一个格子，全部可走才算通视。
    // 正好穿过格点时按对角通过（与本项目允许斜穿拐角的移动规则一致）。整数运算。
    public static bool LineOfSight(GridMap map, int x0, int y0, int x1, int y1)
    {
        if (!map.IsWalkable(x0, y0))
            return false;

        int nx = Math.Abs(x1 - x0);
        int ny = Math.Abs(y1 - y0);
        int signX = Math.Sign(x1 - x0);
        int signY = Math.Sign(y1 - y0);
        int x = x0;
        int y = y0;
        int ix = 0;
        int iy = 0;

        while (ix < nx || iy < ny)
        {
            long decision = (1L + 2 * ix) * ny - (1L + 2 * iy) * nx;

            if (decision == 0)
            {
                x += signX;
                y += signY;
                ix++;
                iy++;
            }
            else if (decision < 0)
            {
                x += signX;
                ix++;
            }
            else
            {
                y += signY;
                iy++;
            }

            if (!map.IsWalkable(x, y))
                return false;
        }

        return true;
    }
}
