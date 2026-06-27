using System;

namespace JPS.Pathfinding
{
    /// <summary>
    /// JPS 跳点 / 强迫邻居判定规则。默认禁止斜穿角（no-corner-cutting）；
    /// 定义条件编译符号 JPS_ALLOW_CORNER_CUTTING 可恢复“允许斜穿拐角”。
    /// 用一个 walkable 委托抽象“可走”判定，规则在预计算/运行时保持一致。
    /// </summary>
    internal static class JpsRules
    {
        public static bool IsJumpPoint(Func<int, int, bool> walkable, int x, int y, int dx, int dy)
        {
            if (JpsDirections.IsDiagonal(dx, dy))
                return HasDiagonalForcedNeighbor(walkable, x, y, dx, dy);

            return HasCardinalForcedNeighbor(walkable, x, y, dx, dy);
        }

        // ──────────────────────────────────────────────────────────────────────────
        // 强迫邻居（forced neighbor）：x 旁边有障碍时，“不经过 x 绕过去”的那条路被挡住，
        // 某个邻居只能经过 x 才能到达 → 它被“强迫”保留，x 因此成为一个跳点（必须入队）。
        // （图中：p=父，x=当前，#=障碍，F=强迫邻居，·=普通；以向右移动 → 为例。）
        // ──────────────────────────────────────────────────────────────────────────

        // 对角移动 (dx,dy) 到达 (x,y) 时的强迫邻居。
        //
        // 切角版（JPS_ALLOW_CORNER_CUTTING）：对角到 x 时，某个正交“身后”格被挡、其对应
        // 斜向可走 → 该斜向是强迫邻居（切角能斜穿过去，但不经 x 绕不过）。
        //
        // 不切角版（默认，SoCS'12）：对角**不产生**强迫邻居。因为不许切角时，要对角走到 x，
        // 它的两个正交邻居 (x-dx,y)、(x,y-dy) 本就必须可走（否则就是切角，非法），
        // “旁边正交格被挡”这个前提不成立 → 恒 false。
        //     p ·          要从 p(左上) 对角到 x，左邻和上邻必须都开，
        //     · x          所以障碍逼出的转向只会由“直线移动”的强迫邻居捕获。
        // 论文原文："only straight steps from p to x may produce forced neighbours"。
        public static bool HasDiagonalForcedNeighbor(Func<int, int, bool> w, int x, int y, int dx, int dy)
        {
#if JPS_ALLOW_CORNER_CUTTING
            // 允许斜穿角：旁边正交格被挡、对应斜向可走即强迫邻居
            return (!w(x - dx, y) && w(x - dx, y + dy)) ||
                   (!w(x, y - dy) && w(x + dx, y - dy));
#else
            // 禁止斜穿角（Harabor & Grastien, SoCS'12）：对角移动不产生强迫邻居。
            // 论文明确："only straight steps from p to x may produce forced neighbours"。
            return false;
#endif
        }

        // 直线移动时的强迫邻居。两种模型下，强迫邻居出现的位置不同：
        //
        // 切角版（JPS_ALLOW_CORNER_CUTTING）——强迫邻居在“斜前方”：
        //     · # F        正上 # 挡住绕行，到右上 F 只能经 x 斜穿墙角（切角）
        //     p x →        规则: !w(x, y±1) && w(x+dx, y±1)
        //     · · ·
        //
        // 不切角版（默认，SoCS'12）——强迫邻居在“正侧方 / 墙刚结束处”：
        //     # F ·        身后上方 # 是墙、当前上方 F 刚露出；不许切角时只能 x→F 竖直过去
        //     p x →        规则: w(x, y±1) && !w(x-dx, y±1)
        //     · · ·
        public static bool HasCardinalForcedNeighbor(Func<int, int, bool> w, int x, int y, int dx, int dy)
        {
#if JPS_ALLOW_CORNER_CUTTING
            // 允许斜穿角：当前侧被挡、其斜前方可走 → 斜前方是强迫邻居
            if (dy == 0)
            {
                return (!w(x, y + 1) && w(x + dx, y + 1)) ||
                       (!w(x, y - 1) && w(x + dx, y - 1));
            }

            return (!w(x + 1, y) && w(x + 1, y + dy)) ||
                   (!w(x - 1, y) && w(x - 1, y + dy));
#else
            // 禁止斜穿角：强迫邻居是“墙刚结束”的正交格——身后侧被挡、当前侧可走，
            // 该侧正交格只能经过 (x,y) 到达（绕开身后的墙）。
            if (dy == 0)   // 水平移动
            {
                return (w(x, y + 1) && !w(x - dx, y + 1)) ||
                       (w(x, y - 1) && !w(x - dx, y - 1));
            }

            return (w(x + 1, y) && !w(x + 1, y - dy)) ||
                   (w(x - 1, y) && !w(x - 1, y - dy));
#endif
        }
    }
}
