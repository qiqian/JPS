using JPS.Models;

namespace JPS.Pathfinding;

/// <summary>
/// 视线拉直（string-pulling）路径平滑：在保证连线不穿过阻挡的前提下，
/// 贪心地把路径上能直连的点之间的拐点去掉，得到更短更顺的折线。
/// </summary>
public static class PathSmoother
{
    public static List<(int X, int Y)> Smooth(GridMap map, List<(int X, int Y)> path)
    {
        var result = new List<(int X, int Y)>();
        if (path.Count == 0)
            return result;

        result.Add(path[0]);

        int i = 0;
        while (i < path.Count - 1)
        {
            // 从当前锚点出发，找能直视到的最远点
            int j = path.Count - 1;
            while (j > i + 1 && !LineOfSight(map, path[i].X, path[i].Y, path[j].X, path[j].Y))
                j--;

            result.Add(path[j]);
            i = j;
        }

        return result;
    }

    // 超覆盖(supercover)直线视线检测：遍历线段经过的每一个格子，全部可走才算通视。
    // 正好穿过格点时按对角通过（与本项目允许斜穿拐角的移动规则一致）。
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
