using System;
using System.Collections.Generic;
using JPS.Models;
#if UNITY_2022_1_OR_NEWER
using Vector2 = UnityEngine.Vector2;   // Unity 2022+：复用引擎自带的 Vector2
#else
using Vector2 = System.Numerics.Vector2;   // .NET / WinForms 构建
#endif

namespace JPS.Pathfinding
{
    /// <summary>
    /// 前向增量视线拉直（forward-incremental string pulling）路径平滑。
    /// 从当前锚点出发不断向前延伸，直到与某点失去视线，就把上一个可见点定为新锚点。
    /// 每个路径点只做一次视线检测，最坏复杂度 O(n·L)。
    /// 输出为连续格坐标（格中心 = cx+0.5），仅用于显示，不参与整数寻路。
    /// </summary>
    public static class PathSmoother
    {
        public static List<Vector2> Smooth(GridMap map, List<(int X, int Y)> path)
        {
            var result = new List<Vector2>();
            if (path.Count == 0)
                return result;

            // 起点一定保留
            result.Add(Center(path[0]));
            if (path.Count == 1)
                return result;

            int anchor = 0;
            // 注意从 i=2 开始：锚点与其相邻点(anchor+1)必然通视，无需检测
            for (int i = 2; i < path.Count; i++)
            {
                // 锚点到 path[i] 失去视线：path[i-1] 是从锚点出发仍可见的最远点，定为新拐点
                if (!LineOfSight(map, path[anchor].X, path[anchor].Y, path[i].X, path[i].Y))
                {
                    result.Add(Center(path[i - 1]));
                    anchor = i - 1;
                }
            }

            // 终点一定保留
            result.Add(Center(path[path.Count - 1]));
            return result;
        }

        /// <summary>格 (cx,cy) 的中心点，连续坐标。</summary>
        private static Vector2 Center((int X, int Y) c) => new Vector2(c.X + 0.5f, c.Y + 0.5f);

        /// <summary>
        /// 超覆盖(supercover)直线视线检测：用整数增量遍历线段经过的每一个格子，全部可走才算通视。
        /// 决策量 decision 比较“下一步水平走还是垂直走更贴近真实直线”：
        ///   &lt;0 走水平，&gt;0 走垂直，==0 表示正好穿过格点 → 按对角同时走。
        ///   对角穿越是否检查两侧由 JPS_ALLOW_CORNER_CUTTING 控制，与寻路移动规则保持一致。
        /// 全整数运算，可被整数寻路安全复用。
        /// </summary>
        public static bool LineOfSight(GridMap map, int x0, int y0, int x1, int y1)
        {
            if (!map.IsWalkable(x0, y0))
                return false;

            int nx = Math.Abs(x1 - x0);   // 水平总步数
            int ny = Math.Abs(y1 - y0);   // 垂直总步数
            int signX = Math.Sign(x1 - x0);
            int signY = Math.Sign(y1 - y0);
            int x = x0;
            int y = y0;
            int ix = 0;   // 已走的水平步数
            int iy = 0;   // 已走的垂直步数

            while (ix < nx || iy < ny)
            {
                // decision = (2*ix+1)*ny - (2*iy+1)*nx，比较水平/垂直推进的相对进度
                long decision = (1L + 2 * ix) * ny - (1L + 2 * iy) * nx;

                if (decision == 0)
                {
                    // 正好穿过格点：对角推进
                    x += signX;
                    y += signY;
                    ix++;
                    iy++;
#if !JPS_ALLOW_CORNER_CUTTING
                    // 默认禁止斜穿角：对角穿越的两个共角格不能是阻挡（与寻路移动规则一致）
                    if (!map.IsWalkable(x - signX, y) || !map.IsWalkable(x, y - signY))
                        return false;
#endif
                }
                else if (decision < 0)
                {
                    x += signX;   // 水平推进
                    ix++;
                }
                else
                {
                    y += signY;   // 垂直推进
                    iy++;
                }

                if (!map.IsWalkable(x, y))
                    return false;   // 经过的某格被挡 → 不通视
            }

            return true;
        }
    }
}
