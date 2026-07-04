/*
 * PathSmoother.cs
 * JPS Pathfinding
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

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
    internal static class PathSmoother
    {
        internal static List<Vector2> Smooth(GridMap map, List<(int X, int Y)> path)
        {
            var result = new List<Vector2>();
            if (path.Count == 0)
                return result;

            // 起点一定保留
            result.Add(Center(path[0]));
            if (path.Count == 1)
                return result;

            var anchor = path[0];
            var previous = path[0];

            // 输入是 compact path，不额外构造 expanded path；但平滑判断仍沿每个 compact 段逐格前进。
            // anchor -> previous 是已知可通的原路径前缀，只测试 anchor 能否继续跨到 current。
            for (int segment = 1; segment < path.Count; segment++)
            {
                var from = path[segment - 1];
                var to = path[segment];
                int stepX = Math.Sign(to.X - from.X);
                int stepY = Math.Sign(to.Y - from.Y);
                int steps = Math.Max(Math.Abs(to.X - from.X), Math.Abs(to.Y - from.Y));
                int nextStep = 1;

                while (nextStep <= steps)
                {
                    if (previous == anchor)
                    {
                        previous = to;
                        break;
                    }

                    if (LineOfSight(map, anchor.X, anchor.Y, to.X, to.Y))
                    {
                        previous = to;
                        break;
                    }

                    int lo = nextStep;
                    int hi = steps;
                    int best = nextStep - 1;

                    while (lo <= hi)
                    {
                        int mid = lo + ((hi - lo) >> 1);
                        var probe = (X: from.X + stepX * mid, Y: from.Y + stepY * mid);
                        if (LineOfSight(map, anchor.X, anchor.Y, probe.X, probe.Y))
                        {
                            best = mid;
                            lo = mid + 1;
                        }
                        else
                        {
                            hi = mid - 1;
                        }
                    }

                    var bestPoint = (X: from.X + stepX * best, Y: from.Y + stepY * best);
                    result.Add(Center(bestPoint));
                    anchor = bestPoint;

                    int failStep = best + 1;
                    if (failStep > steps)
                    {
                        previous = bestPoint;
                        nextStep = failStep;
                    }
                    else
                    {
                        previous = (X: from.X + stepX * failStep, Y: from.Y + stepY * failStep);
                        nextStep = failStep + 1;
                    }
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
        internal static bool LineOfSight(GridMap map, int x0, int y0, int x1, int y1)
        {
            if (!map.IsWalkable(x0, y0))
                return false;

            int nx = Math.Abs(x1 - x0);   // 水平总步数
            int ny = Math.Abs(y1 - y0);   // 垂直总步数
            int signX = Math.Sign(x1 - x0);
            int signY = Math.Sign(y1 - y0);
            int x = x0;
            int y = y0;

            if (nx == ny && nx != 0)
            {
                while (x != x1)
                {
                    x += signX;
                    y += signY;
#if !JPS_ALLOW_CORNER_CUTTING
                    if (!map.IsWalkable(x - signX, y) || !map.IsWalkable(x, y - signY))
                        return false;
#endif
                    if (!map.IsWalkable(x, y))
                        return false;
                }
                return true;
            }

            int ix = 0;   // 已走的水平步数
            int iy = 0;   // 已走的垂直步数
            long decision = (long)ny - nx;
            long stepX = 2L * ny;
            long stepY = 2L * nx;
            long stepDiag = 2L * (ny - nx);

            while (ix < nx || iy < ny)
            {
                if (decision == 0)
                {
                    // 正好穿过格点：对角推进
                    x += signX;
                    y += signY;
                    ix++;
                    iy++;
                    decision += stepDiag;
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
                    decision += stepX;
                }
                else
                {
                    y += signY;   // 垂直推进
                    iy++;
                    decision -= stepY;
                }

                if (!map.IsWalkable(x, y))
                    return false;   // 经过的某格被挡 → 不通视
            }

            return true;
        }
    }
}
